//
// Copyright (c) 2018, IRIS Laboratory, IEETA, University of Aveiro - Portugal
//
// @date    2018-10-30
// @authors Eurico Pedrosa <efp@ua.pt>
//

#include <iostream>
#include <fstream>
#include <vector>

#include <boost/program_options.hpp>

#include "rtk/print.h"

#include <rtk/structs/laser_scan.h>
#include <rtk/slam/slam2d.h>

#include <rtk/time/rate.h>
#include <rtk/time/timer.h>
#include <rtk/time/frequency.h>

#include "carmen.h"

using namespace std;
using namespace rtk;
using namespace rtk::slam;
using namespace rtk::dataset;

namespace po = boost::program_options;

static const char kVersion[]=
"Slam2D by Eurico Pedrosa <efp@ua.pt> \n\
IRIS lab, IEETA/DETI, University of Aveiro, 2015-2016.";

static const char kUsage[] =
"usage: slam2d [options] <logname>\n\
\n\
Options:\n\
  -t <value> --translation=<value>      Accumulated translation before update [default: 0.01]\n\
  -a <value> --rotation=<value>         Accumulated angular motion before update [default: 0.2]\n\
  -d <value> --max-distance=<value>     Maximum Euclidean distance [default: 0.5]\n\
  -x <value> --truncated-ray=<value>    Truncated ray say [default: 0.0 (off)]\n\
  -r <value> --resolution=<value>       Resolution of the maps [default: 0.05]\n\
  -i <value> --max-iterations=<value>   Maximum iteration in the optimization [default: 100]\n\
  -p <value> --patch-size=<value>       The size (in cells) of each individual patch [default: 32]\n\
  -s <name> --strategy=<name>           The optimization strategy to use [default: gn]\n\
                                        Currently available strategies are Levenberg-Marquard (lm),\n\
                                        Gauss-Newton (gn) and Dogleg (dl)\n\
  -c --compress                         Compress patches than are not in cache\n\
  -m <value> --max-range <value>        Maximum range (in meters) of a laser scan [default: 40.0]\n\
  -l <name> --log <name>                Write the corrected trajectory to a log file\n\
  --rate <value>                        Call rate of the slam update.\n\
  --summary                             Show the execution summary before leaving\n\
  --save-occ <name>                     Save the resulting occupancy map\n\
  --save-dm <name>                      Save a map that depicts the distance map\n\
  --save-metrics <name>                 Save execution metrics (per slam update)\n\
  -q --quiet                            Do not show status bar\n\
\n\
  -h --help                             Show this screen\n\
  --version                             Show version";

class Slam2DApp {
public:

    Slam2DApp(int argc, const char* argv[])
    {
        po::options_description description("Options");
        description.add_options()
            ("translation,t",    po::value<double>(&options_.trans_thresh)->default_value(0.01), "")
            ("rotation,a",       po::value<double>(&options_.rot_thresh)->default_value(0.2), "")
            ("max-distance,d",   po::value<double>(&options_.l2_max)->default_value(0.5), "")
            ("truncated-ray,x",  po::value<double>(&options_.truncated_ray)->default_value(0.0), "")
            ("resolution,r",     po::value<double>(&options_.resolution)->default_value(0.05), "")
            ("max-iterations,i", po::value<uint32_t>(&options_.max_iter)->default_value(100), "")
            ("patch-size,p",     po::value<uint32_t>(&options_.patch_size)->default_value(32), "")
            ("strategy,s",       po::value<string>(&options_.strategy)->default_value("gn"), "")
            ("compress,c",       po::value<string>(&calgorithm_)->default_value(""), "")
            ("max-range,m",      po::value<double>(&range_max_)->default_value(50), "")
            ("rate",             po::value<double>(&rate_)->default_value(-1), "")
            ("log,l",            po::value<string>(&log_out_)->default_value(""), "")
            ("save-occ",         po::value<string>(&occ_out_)->default_value(""), "")
            ("save-dm",          po::value<string>(&dm_out_)->default_value(""), "")
            ("save-metrics",     po::value<string>(&metrics_out_)->default_value(""), "")
            ("logname",          po::value<string>(&log_in_), "")
            ("summary",          po::bool_switch(&keep_summary_), "")
            ("quiet,q",          po::bool_switch(&quiet_), "")
            ("help,h",  "")
            ("version", "");

        po::positional_options_description p;
        p.add("logname", -1);

        po::variables_map options;
        try{
            po::store(po::command_line_parser(argc, argv).options(description).positional(p).run(),
                      options);
            po::notify(options);
        } catch (po::error& e) {
            cout << e.what() << std::endl;
            exit(1);
        }

        if (options.count( "help" ) or not options.count("logname")) {
            cout << kUsage << std::endl;
            exit(0);
        }

        if (options.count( "version" )) {
            cout << kVersion << std::endl;
            exit(0);
        }

        if (calgorithm_ != "")
            use_compression_ = true;
        else
            use_compression_ = false;

    }

    void run()
    {
        options_.keep_summary = keep_summary_ or (not metrics_out_.empty());

        Slam2D slam(options_);
        slam.useCompression(use_compression_, calgorithm_);

        CarmenWriter carmen;
        bool write_log = false;
        if (not log_out_.empty()) {
            if (not carmen.open(log_out_)){
                std::cerr << "Unable to open log file " << log_out_ << std::endl;
                exit(1);
            }
            write_log = true;
        }// end if

        // 1. Load log file
        std::cout << "Loading... "; std::flush(std::cout);
        vector<LaserScan> laser_scans;
        loadCarmenLog(log_in_, laser_scans);

        rtk::time::Rate      rate(rate_);
        rtk::time::Frequency data_frequency(100);
        rtk::time::Frequency update_frequency;
        rtk::time::Timer     time(true);
        int count = 0;
        for (size_t i = 0; i < laser_scans.size(); ++i){

            LaserScan& ls = laser_scans[i];
            ls.range_max = 30.0;

            // estimate data frequency
            data_frequency.event(ls.stamp);

            PointCloudXYZ::Ptr cloud = ls.toPointCloud();
            rtk::geom::Pose2D odom(ls.location.x(), ls.location.y(), ls.location.yaw());

            bool updated = slam.update(cloud, odom, ls.stamp);

            if (not updated)
                // we only sleep after an update. This way dry updates are skipped.
                continue;
            //--
            rate.sleep();

            count++;

            if (write_log)
                carmen.writePose(slam.getPose(), ls.stamp);

            if (not quiet_){
                update_frequency.event();

                double elapsed = time.elapsed().toSec();
                double eta = (elapsed * laser_scans.size() / (i+1)) - elapsed;
                std::string statusline;

                uint64_t totalmem, occmem, dmmem;
                uint32_t proccesed_cells;

                totalmem = slam.getMemoryUsage(occmem, dmmem);
                proccesed_cells = slam.getNumberOfProcessedCells();
                rtk::print("\r\33[2K[t %.2f] [a %.2f] [d %.2f] [s %s] [f %d/%4d Hz] "
                           "[@ %ld/%ld] [\u0394 %4d] [mem %.3f (%.3f/%.3f) MB] [elapsed %d:%02d] [eta %d:%02d]",
                           options_.trans_thresh, options_.rot_thresh, options_.l2_max,
                           options_.strategy.c_str(),
                           (int)data_frequency.getFrequency(),
                           (int)update_frequency.getFrequency(),
                           i+1, laser_scans.size(), proccesed_cells,
                           (occmem + dmmem) / 1024.0 / 1024.0,
                           occmem / 1024.0 / 1024.0,
                           dmmem / 1024.0 / 1024.0,
                           (int)elapsed/60, (int)elapsed % 60,
                           (int)eta/60, (int)eta%60);

                std::cout << std::flush;
            }
        } // end for

        std::cout << std::endl;

        if (keep_summary_)
            std::cout << slam.getSummary()->fullReport() << std::endl;

        if (not metrics_out_.empty()){
            slam.getSummary()->saveMetrics(metrics_out_);
        } // end if

        if (not occ_out_.empty())
            slam.saveOccImage(occ_out_);

        if (not dm_out_.empty())
            slam.saveDistImage(dm_out_);
    }

//==============================================================
    void loadCarmenLog(const string& log, vector<LaserScan>& laser_scans)
    {
        CarmenReader carmen;
        if (not carmen.open(log)){
            std::cerr << "Unable to open log file " << log << std::endl;
            exit(1);
        }

        LaserScan ls;
        while( carmen.readLaserScan(&ls) )
            laser_scans.push_back(ls);
    }

private:
    Slam2D::Options options_;

    double rate_;
    double range_max_;

    string log_in_;
    string log_out_;
    string occ_out_;
    string dm_out_;
    string metrics_out_;

    bool quiet_;
    bool keep_summary_;
    bool use_compression_;
    std::string calgorithm_;
};


int main(int argc, const char *argv[])
{
    Slam2DApp app(argc, argv);
    app.run();

    return 0;
}

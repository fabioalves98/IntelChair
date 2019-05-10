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
#include <rtk/slam/pf_slam2d.h>

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
"Particle Filter Slam2D by Eurico Pedrosa <efp@ua.pt> \n\
IRIS lab, IEETA/DETI, University of Aveiro, 2015-2016.";

static const char kUsage[] =
"usage: slam2d [options] <logname>\n\
\n\
Options:\n\
  -t <value> --translation=<value>      Accumulated translation before update [default: 1.0]\n\
  -a <value> --rotation=<value>         Accumulated angular motion before update [default: 0.5]\n\
  -d <value> --max-distance=<value>     Maximum Euclidean distance [default: 0.5]\n\
  -x <value> --truncated-ray=<value>    Truncated ray say [default: 0.0 (off)]\n\
  -r <value> --resolution=<value>       Resolution of the maps [default: 0.05]\n\
  -i <value> --max-iterations=<value>   Maximum iteration in the optimization [default: 100]\n\
  -p <value> --particles                The number of particles to use [default: 30]\n\
  -g <value> --lgain                    Likelihood smooth gain [default: 3]\n\
  -s <name> --strategy=<name>           The optimization strategy to use [default: gn]\n\
                                        Currently available strategies are Levenberg-Marquard (lm),\n\
                                        Gauss-Newton (gn) and Dogleg (dl)\n\
  -c <algo> --compress <algo>           Compress patches than are not in cache using the algorithm <algo>\n\
                                        Available algorithms are: zlib, zstd, lz4, quicklz and snappy\n\
  -m <value> --max-range <value>        Maximum range (in meters) of a laser scan [default: 25.0]\n\
  -l <name> --log <name>                Write the corrected trajectory to a log file\n\
  -L <value --patch-size=<value>        The size (in cells) of each individual patch [default: 32]\n\
  --rate <value>                        Call rate of the slam update.\n\
  --threads <value>                     Number of threads to use [default: -1]\n\
  --seed <value>                        Pseudo random generator seed value [default: 0]\n\
  --summary                             Show the execution summary before leaving\n\
  --save-occ <name>                     Save the resulting map\n\
  --save-patch <name>                   Save a map that depicts the allocated patches\n\
  --save-metrics <name>                 Save execution metrics (per slam update)\n\
  -q --quiet                            Do not show status bar\n\
\n\
  -h --help                             Show this screen\n\
  --version                             Show version";

class PFSlam2DApp {
public:

    PFSlam2DApp(int argc, const char* argv[])
    {
        po::options_description description("Options");
        description.add_options()
            ("translation,t",    po::value<double>(&options_.trans_thresh)->default_value(1.0), "")
            ("rotation,a",       po::value<double>(&options_.rot_thresh)->default_value(0.5), "")
            ("max-distance,d",   po::value<double>(&options_.l2_max)->default_value(0.5), "")
            ("truncated-ray,x",  po::value<double>(&options_.truncated_ray)->default_value(0.0), "")
            ("resolution,r",     po::value<double>(&options_.resolution)->default_value(0.05), "")
            ("max-iterations,i", po::value<uint32_t>(&options_.max_iter)->default_value(100), "")
            ("particles,p",      po::value<uint32_t>(&options_.particles)->default_value(30), "")
            ("lgain,g",          po::value<double>(&options_.meas_sigma_gain)->default_value(3), "")
            ("patch-size,L",     po::value<uint32_t>(&options_.patch_size)->default_value(32), "")
            ("strategy,s",       po::value<string>(&options_.strategy)->default_value("gn"), "")
            ("compress,c",       po::value<string>(&options_.calgorithm)->default_value(""), "")
            ("cache-size,C",     po::value<uint32_t>(&options_.cache_size)->default_value(50), "")
            ("max-range,m",      po::value<double>(&range_max_)->default_value(25), "")
            ("rate",             po::value<double>(&rate_)->default_value(-1), "")
            ("threads",          po::value<int32_t>(&options_.threads)->default_value(-1), "")
            ("seed",             po::value<uint32_t>(&options_.seed)->default_value(0), "")
            ("log,l",            po::value<string>(&log_out_)->default_value(""), "")
            ("save-occ",         po::value<string>(&occ_out_)->default_value(""), "")
            ("save-patch",       po::value<string>(&patch_out_)->default_value(""), "")
            ("save-metrics",     po::value<string>(&metrics_out_)->default_value(""), "")
            ("logname",          po::value<string>(&log_in_), "")
            ("summary",          po::bool_switch(&keep_summary_), "")
            ("quiet",            po::bool_switch(&quiet_), "")
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

        //
        if (options_.calgorithm != "")
            options_.use_compression = true;

    }

    void run()
    {
        PFSlam2D slam(options_);
        PFSlam2D::Summary summary;

        // the seed value can change !!
        options_ = slam.getOptions();

        if (keep_summary_ or not metrics_out_.empty())
            slam.setSummary(&summary);

        // 1. Load log file
        if (not quiet_)
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
            ls.range_max = range_max_;

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

            if (not quiet_){
                update_frequency.event();

                double elapsed = time.elapsed().toSec();
                double eta = (elapsed * laser_scans.size() / (i+1)) - elapsed;
                std::string statusline;

                uint64_t totalmem, occmem, dmmem;
                totalmem = slam.getMemoryUsage(occmem, dmmem);
                rtk::print("\r\33[2K[seed %u] [t %.2f] [a %.2f] [d %.2f] [s %s] [f %d/%3d Hz] "
                           "[@ %ld/%ld] [neff %.2f] [mem %7.2f (%.2f/%.2f) MB] [elapsed %d:%02d] [eta %d:%02d]",
                           options_.seed, options_.trans_thresh, options_.rot_thresh, options_.l2_max,
                           options_.strategy.c_str(),
                           (int)data_frequency.getFrequency(),
                           (int)update_frequency.getFrequency(),
                           i+1, laser_scans.size(),
                           slam.getNeff(),
                           (occmem + dmmem) / 1024.0 / 1024.0,
                           occmem /  1024.0 / 1024.0,
                           dmmem /   1024.0 / 1024.0,
                           (int)elapsed/60, (int)elapsed % 60,
                           (int)eta/60, (int)eta%60);

                std::cout << std::flush;
            }
        } // end for

        std::cout << std::endl;

        if (not log_out_.empty()) {

            CarmenWriter carmen;
            if (not carmen.open(log_out_)){
                std::cerr << "Unable to open log file " << log_out_ << std::endl;
                exit(1);
            }

            // Save parameters to the log file
            const PFSlam2D::Options& options = slam.getOptions();
            carmen.writeParam("particles", options.particles);
            carmen.writeParam("srr", options.srr);
            carmen.writeParam("str", options.str);
            carmen.writeParam("stt", options.stt);
            carmen.writeParam("srt", options.srt);
            carmen.writeParam("meas_sigma", options.meas_sigma);
            carmen.writeParam("meas_sigma_gain", options.meas_sigma_gain);
            carmen.writeParam("trans_thresh", options.trans_thresh);
            carmen.writeParam("rot_thresh", options.rot_thresh);
            carmen.writeParam("l2_max", options.l2_max);
            carmen.writeParam("resolution", options.resolution);
            carmen.writeParam("patch_size", options.patch_size);
            carmen.writeParam("max_iter", options.max_iter);
            carmen.writeParam("strategy", options.strategy);
            carmen.writeParam("threads", options.threads);
            carmen.writeParam("seed", options.seed);
            carmen.writeParam("use_compression", options.use_compression);

            // save the trajectories
            uint32_t idx = slam.getBestParticleIdx();
            const size_t num_poses = slam.getParticles()[idx].poses.size();
            for (size_t i = 0; i < num_poses; ++i){
                carmen.writePose(slam.getParticles()[idx].poses[i],
                                 slam.getTimestamps()[i]);

            } // end for
        }// end if

        if (keep_summary_){
            std::cout << summary.fullReport() << std::endl;
        } // end if keep_summary_

        if (not metrics_out_.empty()){
            summary.saveMetrics(metrics_out_);
        } // end if

        if (not occ_out_.empty())
            slam.saveOccImage(occ_out_);
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
    PFSlam2D::Options options_;

    double rate_;
    double range_max_;

    string log_in_;
    string log_out_;
    string occ_out_;
    string patch_out_;
    string metrics_out_;

    bool keep_summary_;
    bool quiet_;
};


int main(int argc, const char *argv[])
{
    PFSlam2DApp app(argc, argv);
    app.run();

    return 0;
}

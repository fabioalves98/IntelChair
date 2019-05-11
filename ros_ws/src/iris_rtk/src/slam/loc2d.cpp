//
// Copyright (c) 2018, IRIS Laboratory, IEETA, University of Aveiro - Portugal
//
// @date    2018-10-30
// @authors Eurico Pedrosa <efp@ua.pt>
//

#include <iostream>
#include <fstream>

#include "rtk/print.h"

#include <rtk/nlls/gauss_newton.h>
#include <rtk/nlls/levenberg_marquardt.h>
#include <rtk/nlls/dog_leg.h>
#include <rtk/nlls/rprop.h>

#include <rtk/slam/loc2d.h>
#include <rtk/slam/match_surface_2d.h>

#include <rtk/geom/line_segment.h>

#include <rtk/time/timer.h>
#include <rtk/math/stats.h>
#include <rtk/math/random.h>

namespace rtk  {
namespace slam {

std::string Loc2D::Summary::fullReport() const
{
    std::string report;

    report = format("Loc2D Report\n"
                    "=============\n\n");

    math::Stats sa_up; sa_up(time_update);

    double timespan  = timestamps.back() - timestamps.front();
    double fulltime  = sa_up.sum();
    report += format("Number of updates    %ld\n"
                     "Problem time span    %d minutes and %d seconds\n"
                     "Execution time span  %d minutes and %d seconds\n"
                     "Execution frequency  %d Hz\n"
                     "Realtime factor      %.2fx\n",
                     timestamps.size(),
                     (uint32_t)timespan / 60, (uint32_t)timespan % 60,
                     (uint32_t)fulltime / 60, (uint32_t)fulltime % 60,
                     (uint32_t)(1.0 / sa_up.mean()),
                     timespan / fulltime);

    uint32_t num_successful_steps   = 0;
    uint32_t num_unsuccessful_steps = 0;

    math::Stats sa_solv;
    math::Stats sa_iters;
    math::Stats sa_rmse;
    for (size_t i = 0; i < solver_summary.size(); ++i){

        sa_solv(solver_summary[i].execution_time);
        sa_iters(solver_summary[i].step_summaries.size());
        sa_rmse(rmse[i]);

        num_successful_steps   += solver_summary[i].num_successful_steps;
        num_unsuccessful_steps += solver_summary[i].num_unsuccessful_steps;
    }

    report += format("\nOptimization\n"
                     "  Strategy                        %s\n"
                     "  Iterations                      %u\n"
                     "    Per step                      %f ± %f [%u, %u]\n"
                     "    Successful                    %.2f %%\n"
                     "    Unsuccessful                  %.2f %%\n"
                     "  RMSE                            %.4f ± %.4f\n",
                     strategy.c_str(), (uint32_t)sa_iters.sum(),
                     sa_iters.mean(), sa_iters.std(),
                     (uint32_t)sa_iters.min(),  (uint32_t)sa_iters.max(),
                     num_successful_steps   / sa_iters.sum() * 100,
                     num_unsuccessful_steps / sa_iters.sum() * 100,
                     sa_rmse.mean(), sa_rmse.std());

    report += format("\nExecution times (mean ± std [min, max, median]) in miliseconds\n"
                     "--------------------------------------------------------------\n"
                     "Update                            %f ± %f [%f, %f, %f]\n"
                     "  Optimization                    %f ± %f [%f, %f, %f]\n",
    sa_up.mean()   *1000, sa_up.std()   *1000, sa_up.min()   *1000, sa_up.max()   *1000, sa_up.median()   *1000,
    sa_solv.mean() *1000, sa_solv.std() *1000, sa_solv.min() *1000, sa_solv.max() *1000, sa_solv.median() *1000);

    return report;
}

bool Loc2D::Summary::saveMetrics(const std::string& filename) const
{

    std::ofstream out;
    out.open(filename.c_str());

    if (not out.is_open())
        return false;

    // Write header
    out << "timestamp,update,scanmatching,rmse,iterations"  << std::endl;

    // write metrics
    const size_t num_data = timestamps.size();
    size_t resample_idx = 0;
    for (size_t i = 1; i < num_data-1; ++i){

        std::string metrics ;
        metrics = format("%.10f,%.10f,%.10f,%.5f,%d",
                         timestamps[i], time_update[i],
                         solver_summary[i].execution_time, rmse[i],
                         solver_summary[i].num_successful_steps + solver_summary[i].num_unsuccessful_steps);

        out << metrics << std::endl;
    } // end for

    out.close();
    return true;
}

Loc2D::Options::Options()
{
    trans_thresh = 0.5;
    rot_thresh   = 0.5;
    l2_max       = 1.0;
    resolution   = 0.05;
    patch_size   = 32;
    max_iter     = 100;
    keep_summary = false;
}


void Loc2D::Init(const Options& options)
{
    occupancy_map = new sdm::SimpleOccupancyMap(options.resolution, options.patch_size, false);
    distance_map  = new sdm::DynamicDistanceMap(options.resolution, options.patch_size, false);
    distance_map->setMaxDistance(options.l2_max);

    /* solver_options_.write_to_stdout= true; */
    solver_options_.max_iterations = options.max_iter;
    solver_options_.strategy       = makeStrategy(options.strategy, Vector2d::Zero());
    /* solver_options_.robust_cost    = makeRobust("cauchy", 0.25); */
    solver_options_.robust_cost.reset(new nlls::CauchyWeight(0.15));

    if (options.keep_summary)
        summary_ = new Loc2D::Summary;
    else
        summary_ = 0;

    trans_thresh_ = options.trans_thresh;
    rot_thresh_   = options.rot_thresh;

    has_first_scan = false;
    do_global_loclization_ = false;
}

Loc2D::~Loc2D()
{}

bool Loc2D::enoughMotion(const rtk::geom::Pose2D& odometry)
{
    if (not has_first_scan)
        return true;

    geom::Pose2D odelta = odom_ - odometry;

    if (odelta.xy().norm() <= trans_thresh_ && std::abs(odelta.rotation()) <= rot_thresh_)
        return false;

    return true;
}

bool Loc2D::update(const PointCloudXYZ::Ptr& surface,
                    const geom::Pose2D& odometry,
                    double timestamp)
{
    rtk::time::Timer timer(true);

    if (not has_first_scan){
        odom_ = odometry;

        has_first_scan = true;
        //-----------------------------------------------------------
        if (summary_){
            summary_->time_update.push_back(timer.elapsed().toSec());
            summary_->timestamps.push_back(timestamp);

            nlls::Solver::Summary solver_summary;
            solver_summary.execution_time = 0.0;
            summary_->solver_summary.push_back(solver_summary);
            summary_->rmse.push_back(0);
        }
        //-----------------------------------------------------------
        return true;
    }

    // 1. Predict from odometry
    geom::Pose2D odelta = odom_ - odometry;
    geom::Pose2D ppose  = pose_ + odelta;

    // only continue if the necessary motion was gathered.
    if (odelta.xy().norm() <= trans_thresh_ &&
        std::abs(odelta.rotation()) <= rot_thresh_)
        return false;

    pose_ = ppose;
    odom_ = odometry;

    if (do_global_loclization_)
        globalLocalization(surface);

    // 2. Optimize
    MatchSurface2D match_surface(distance_map, surface, pose_.se2());

    //-----------------------------------------------------
    if (summary_){
        nlls::Solver::Summary solver_summary;
        nlls::Solve(solver_options_, match_surface, &solver_summary);

        summary_->solver_summary.push_back(solver_summary);
        summary_->rmse.push_back( sqrt((solver_summary.final_cost*2.0) / (surface->points.size()) ) );
    //-----------------------------------------------------
    }else{
        nlls::Solve(solver_options_, match_surface, 0);
    }

    //--
    if (do_global_loclization_){
        VectorXd residuals;
        match_surface.eval(residuals, 0);
        double rmse = sqrt(residuals.squaredNorm()/((double)(surface->points.size() - 1)));

        if (rmse < 0.15)
            do_global_loclization_ = false;
    }
    //--

    pose_.se2() = match_surface.getState();

    //--------------------------------------------
    if (summary_){
        summary_->timestamps.push_back(timestamp);
        summary_->time_update.push_back(timer.elapsed().toSec());
    }
    //-----------------------------------------------------------
    return true;
}

const Loc2D::Summary* Loc2D::getSummary() const
{
    if (summary_ and summary_->strategy.empty())
        summary_->strategy = solver_options_.strategy->name();

    return summary_;
}

void Loc2D::triggerGlobalLocalization()
{
    do_global_loclization_ = true;
}


void Loc2D::globalLocalization(const PointCloudXYZ::Ptr& surface)
{
    Vector3d min, max;
    occupancy_map->bounds(min, max);

    Vector3d diff = max - min;

    double best_error = std::numeric_limits<double>::max();

    const size_t numOfParticles = 3000;
    for (size_t i = 0; i < numOfParticles; ++i){

        double x, y, a;

        for (;;){
            x = min[0] + math::random::uniform() * diff[0];
            y = min[1] + math::random::uniform() * diff[1];

            if (not occupancy_map->isFree(Vector3d(x, y, 0.0)))
                continue;

            a = math::random::uniform() * 2 * M_PI - M_PI;
        }

        geom::Pose2D p(x, y , a);
        VectorXd residuals;
        MatchSurface2D match_surface(distance_map, surface, p.se2());

        match_surface.eval(residuals, 0);

        double error = residuals.squaredNorm();
        if ( error < best_error ){
            best_error = error;
            pose_ = p;
        }
    } // end for

}


Loc2D::StrategyPtr Loc2D::makeStrategy(const std::string& name, const VectorXd& parameters)
{
    if (name == "gn"){
        return StrategyPtr(new nlls::GaussNewton);
    }else if (name == "dl"){
        return StrategyPtr(new nlls::DogLeg);
    }else if (name == "rprop"){
        return StrategyPtr(new nlls::RProp);
    }else {
        return StrategyPtr(new nlls::LevenbergMarquard);
    }
}

Loc2D::RobustCostPtr Loc2D::makeRobust(const std::string& name, const double& param)
{
    if (name == "cauchy")
        return RobustCostPtr(new nlls::CauchyWeight(0.15));
    else if (name == "tstudent")
        return RobustCostPtr(new nlls::TDistributionWeight(3));
    else if (name == "tukey")
        return RobustCostPtr(new nlls::TukeyWeight);
    else
        return RobustCostPtr(new nlls::UnitWeight);
}

}} /* rtk::slam */


//
// Copyright (c) 2018, IRIS Laboratory, IEETA, University of Aveiro - Portugal
//
// @date    2018-10-30
// @authors Eurico Pedrosa <efp@ua.pt>
//

#include <fstream>

#include "rtk/print.h"

#include <rtk/time/timer.h>
#include <rtk/math/stats.h>

#include <rtk/nlls/gauss_newton.h>
#include <rtk/nlls/levenberg_marquardt.h>
#include <rtk/nlls/dog_leg.h>

#include <rtk/slam/slam2d.h>
#include <rtk/slam/match_surface_2d.h>

#include "rtk/sdm/export.h"

namespace rtk  {
namespace slam {

std::string Slam2D::Summary::fullReport() const
{
    std::string report;

    report = format("Slam2D Report\n"
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
    for (size_t i = 0; i < solver_summary.size(); ++i){

        sa_solv(solver_summary[i].execution_time);
        sa_iters(solver_summary[i].step_summaries.size());

        num_successful_steps   += solver_summary[i].num_successful_steps;
        num_unsuccessful_steps += solver_summary[i].num_unsuccessful_steps;
    }

    report += format("\nOptimization\n"
                     "  Strategy                        %s\n"
                     "  Iterations                      %u\n"
                     "    Per step                      %f ± %f [%u, %u]\n"
                     "    Successful                    %.2f %%\n"
                     "    Unsuccessful                  %.2f %%\n",
                     strategy.c_str(), (uint32_t)sa_iters.sum(),
                     sa_iters.mean(), sa_iters.std(),
                     (uint32_t)sa_iters.min(),  (uint32_t)sa_iters.max(),
                     num_successful_steps   / sa_iters.sum() * 100,
                     num_unsuccessful_steps / sa_iters.sum() * 100);

    math::Stats sa_pc_tf; sa_pc_tf(time_pc_tf);
    math::Stats sa_occ;   sa_occ(time_occ);
    math::Stats sa_dm;    sa_dm(time_dm);
    math::Stats sa_mt;    sa_mt(time_mapping);

    report += format("\nExecution times (mean ± std [min, max, median]) in miliseconds\n"
                     "--------------------------------------------------------------\n"
                     "Update                            %f ± %f [%f, %f, %f]\n"
                     "  Optimization                    %f ± %f [%f, %f, %f]\n"
                     "  Mapping                         %f ± %f [%f, %f, %f]\n"
                     "    PointCloud transformation     %f ± %f [%f, %f, %f]\n"
                     "    OccupancyMap update           %f ± %f [%f, %f, %f]\n"
                     "    DistanceMap update            %f ± %f [%f, %f, %f]\n",
    sa_up.mean()   *1000, sa_up.std()   *1000, sa_up.min()   *1000, sa_up.max()   *1000, sa_up.median()   *1000,
    sa_solv.mean() *1000, sa_solv.std() *1000, sa_solv.min() *1000, sa_solv.max() *1000, sa_solv.median() *1000,
    sa_mt.mean()   *1000, sa_mt.std()   *1000, sa_mt.min()   *1000, sa_mt.max()   *1000, sa_mt.median()   *1000,
    sa_pc_tf.mean()*1000, sa_pc_tf.std()*1000, sa_pc_tf.min()*1000, sa_pc_tf.max()*1000, sa_pc_tf.median()*1000,
    sa_occ.mean()  *1000, sa_occ.std()  *1000, sa_occ.min()  *1000, sa_occ.max()  *1000, sa_occ.median()  *1000,
    sa_dm.mean()   *1000, sa_dm.std()   *1000, sa_dm.min()   *1000, sa_dm.max()   *1000, sa_dm.median()   *1000);

    return report;
}

bool Slam2D::Summary::saveMetrics(const std::string& filename) const
{

    std::ofstream out;
    out.open(filename.c_str());

    if (not out.is_open())
        return false;

    // Write header
    out << "timestamp,memory,update,scanmatching,mapping,occ,dm,iterations"  << std::endl;

    // write metrics
    const size_t num_data = timestamps.size();
    size_t resample_idx = 0;
    for (size_t i = 0; i < num_data; ++i){

        auto metrics = format("%.10f,%ld,%.10f,%.10f,%.10f,%.10f,%.10f,%d",
                              timestamps[i], memory[i], time_update[i],
                              solver_summary[i].execution_time, time_mapping[i],
                              time_occ[i], time_dm[i],
                              solver_summary[i].num_successful_steps + solver_summary[i].num_unsuccessful_steps);

        out << metrics << std::endl;
    } // end for

    out.close();
    return true;
}

Slam2D::Options::Options()
{
    trans_thresh = 0.5;
    rot_thresh   = 0.5;
    l2_max       = 1.0;
    truncated_ray= 0.0;
    resolution   = 0.05;
    patch_size   = 20;
    max_iter     = 100;
    keep_summary = false;

    use_compression = false;
    cache_size      = 60;
    calgorithm      = "lz4";
}


Slam2D::Slam2D(const Options& options)
{
    distance_map_ = new sdm::DynamicDistanceMap(options.resolution, options.patch_size);
    distance_map_->setMaxDistance(options.l2_max);

    occupancy_map_ = new sdm::FrequencyOccupancyMap(options.resolution, options.patch_size);

    distance_map_->useCompression(options.use_compression,  options.cache_size, options.calgorithm);
    occupancy_map_->useCompression(options.use_compression, options.cache_size, options.calgorithm);

    /* solver_options_.write_to_stdout= true; */
    solver_options_.max_iterations = options.max_iter;
    solver_options_.strategy       = makeStrategy(options.strategy, Vector2d::Zero());
    /* solver_options_.robust_cost    = makeRobust("cauchy", 0.25); */
    solver_options_.robust_cost.reset(new nlls::CauchyWeight(0.15));

    if (options.keep_summary)
        summary_ = new Slam2D::Summary;
    else
        summary_ = 0;

    trans_thresh_ = options.trans_thresh;
    rot_thresh_   = options.rot_thresh;

    has_first_scan = false;
    number_of_proccessed_cells_ = 0;
    truncated_ray_ = options.truncated_ray;
}

Slam2D::~Slam2D()
{
    delete distance_map_;
    delete occupancy_map_;

    delete summary_;
}

bool Slam2D::enoughMotion(const rtk::geom::Pose2D& odometry)
{
    if (not has_first_scan)
        return true;

    geom::Pose2D odelta = odom_ - odometry;

    if (odelta.xy().norm() <= trans_thresh_ && std::abs(odelta.rotation()) <= rot_thresh_)
        return false;

    return true;
}

bool Slam2D::update(const PointCloudXYZ::Ptr& surface,
                    const geom::Pose2D& odometry,
                    double timestamp)
{
    rtk::time::Timer timer(true);

    if (not has_first_scan){
        odom_ = odometry;
        updateMaps(surface);

        has_first_scan = true;
        //-----------------------------------------------------------
        if (summary_){
            summary_->time_update.push_back(timer.elapsed().toSec());
            summary_->timestamps.push_back(timestamp);
            summary_->memory.push_back(getMemoryUsage());

            nlls::Solver::Summary solver_summary;
            solver_summary.execution_time = 0.0;
            summary_->solver_summary.push_back(solver_summary);
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

    // 2. Optimize
    MatchSurface2D match_surface(distance_map_, surface, pose_.se2());

    //-----------------------------------------------------
    if (summary_){
        nlls::Solver::Summary solver_summary;
        nlls::Solve(solver_options_, match_surface, &solver_summary);
        summary_->solver_summary.push_back(solver_summary);

        //std::cout << solver_summary.fullReport() << std::endl;
    //-----------------------------------------------------
    }else{
        nlls::Solve(solver_options_, match_surface, 0);
    }

    pose_.se2() = match_surface.getState();

    // 3. Update maps
    updateMaps(surface);

    //--------------------------------------------
    if (summary_){
        summary_->timestamps.push_back(timestamp);
        summary_->time_update.push_back(timer.elapsed().toSec());
        summary_->memory.push_back(getMemoryUsage());
    }
    //-----------------------------------------------------------
    return true;
}

const Slam2D::Summary* Slam2D::getSummary() const
{
    if (summary_ and summary_->strategy.empty())
        summary_->strategy = solver_options_.strategy->name();

    return summary_;
}

uint64_t Slam2D::getMemoryUsage() const
{
    uint64_t memory = 0;
    memory += occupancy_map_->memory();
    memory += distance_map_->memory();

    return memory;
}

uint64_t Slam2D::getMemoryUsage(uint64_t& occmem, uint64_t& dmmem) const
{
    occmem = 0;
    dmmem  = 0;

    occmem = occupancy_map_->memory();
    dmmem  = distance_map_->memory();

    return occmem + dmmem;
}

void Slam2D::useCompression(bool compression, const std::string& algorithm)
{
    distance_map_->useCompression(compression,  60, algorithm);
    occupancy_map_->useCompression(compression, 60, algorithm);
}

Slam2D::StrategyPtr Slam2D::makeStrategy(const std::string& name, const VectorXd& parameters)
{
    if (name == "lm"){
        return StrategyPtr(new nlls::LevenbergMarquard);
    }else if (name == "dl"){
        return StrategyPtr(new nlls::DogLeg);
    }else {
        return StrategyPtr(new nlls::GaussNewton);
    }
}

Slam2D::RobustCostPtr Slam2D::makeRobust(const std::string& name, const double& param)
{
    if (name == "cauchy")
        return RobustCostPtr(new nlls::CauchyWeight(0.25));
    else if (name == "tstudent")
        return RobustCostPtr(new nlls::TDistributionWeight(3));
    else if (name == "tukey")
        return RobustCostPtr(new nlls::TukeyWeight);
    else
        return RobustCostPtr(new nlls::UnitWeight);
}

void Slam2D::updateMaps(const PointCloudXYZ::Ptr& surface)
{
    time::Timer timer;
    time::Timer global_timer(true);

    // 1. Transform the point cloud to the model coordinates.
    timer.reset();
    Affine3d moving_tf = Translation3d(surface->sensor_origin_) * surface->sensor_orientation_;
    Affine3d fixed_tf = Translation3d(pose_.x(), pose_.y(), 0.0) * AngleAxisd(pose_.rotation(), Vector3d::UnitZ());

    // the sensor origin (in map coordinates) is the origin
    // for the ray casting.
    Vector3d wso = (fixed_tf * moving_tf).translation();
    Vector3ui so = occupancy_map_->w2m(wso);

    const size_t num_points = surface->points.size();
    Affine3d tf = fixed_tf * moving_tf;

    //-----------------------------------------------------
    if (summary_)
        summary_->time_pc_tf.push_back(timer.elapsed().toSec());
    //-----------------------------------------------------

    // 2. generate the free and occupied positions.
    timer.reset();

    VectorVector3ui free;
    // generate the ray casts
    for (size_t i = 0; i < num_points; ++i){
        Vector3d start = wso;
        Vector3d hit = tf * surface->points[i];

        if (truncated_ray_ > 0.0){
            Vector3d AB = hit - wso;
            double truncate_size = std::min(AB.norm(), truncated_ray_);
            start = hit - AB.normalized() * truncate_size;
        }

        Vector3ui mhit = occupancy_map_->w2m(hit);

        bool changed = occupancy_map_->setOccupied(mhit);
        if ( changed ) distance_map_->addObstacle(mhit);

        occupancy_map_->computeRay(occupancy_map_->w2m(start), mhit, free);
    }

    const size_t num_free = free.size();
    for (size_t i = 0; i < num_free; ++i){
        bool changed = occupancy_map_->setFree(free[i]);
        if ( changed ) distance_map_->removeObstacle(free[i]);
    }

    //-----------------------------------------------------
    if (summary_)
        summary_->time_occ.push_back(timer.elapsed().toSec());
    //-----------------------------------------------------

    // 3. Update the distance map
    timer.reset();
    number_of_proccessed_cells_ = distance_map_->update();
    //---..--------------------------------------------------------------
    if (summary_){
        summary_->time_dm.push_back(timer.elapsed().toSec());
        summary_->time_mapping.push_back(global_timer.elapsed().toSec());
    }//------------------------------------------------------------------
}

}} /* rtk::slam */


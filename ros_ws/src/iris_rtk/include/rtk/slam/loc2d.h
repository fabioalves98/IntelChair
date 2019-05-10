//
// Copyright (c) 2018, IRIS Laboratory, IEETA, University of Aveiro - Portugal
//
// @date    2018-10-30
// @authors Eurico Pedrosa <efp@ua.pt>
//

#pragma once

#include <rtk/types.h>

#include <rtk/math/stats.h>
#include <rtk/geom/pose2d.h>

#include <rtk/nlls/solver.h>

#include <rtk/sdm/dynamic_distance_map.h>
#include <rtk/sdm/simple_occupancy_map.h>

namespace rtk  {
namespace slam {

class Loc2D {
public:

    typedef rtk::nlls::Solver::Options SolverOptions;
    typedef rtk::nlls::Solver::Summary SolverSummary;

    typedef rtk::math::Stats StatsAccumulator;

    typedef rtk::nlls::Strategy::Ptr   StrategyPtr;
    typedef rtk::nlls::RobustCost::Ptr RobustCostPtr;

    typedef boost::shared_ptr<sdm::DynamicDistanceMap>    DynamicDistanceMapPtr;
    typedef boost::shared_ptr<sdm::SimpleOccupancyMap>    SimpleOccupancyMapPtr;

public:

    struct Summary {
        Summary(){}

        std::string fullReport() const;

        bool saveMetrics(const std::string& filename) const;

        ///
        std::vector<double> timestamps;

        /// Holds a summary for each optimization.
        std::vector<SolverSummary> solver_summary;

        /// Memory usage statistics
        //StatsAccumulator stats_memory_usage;
        std::vector<size_t> memory;

        /// Update execution time
        std::vector<double> time_update;

        /// Root Mean Squared Error
        std::vector<double> rmse;

        /// The name of the optimization strategy
        std::string strategy;
    };

public:

    struct Options {
        Options();
        /// the ammount of displacement that the system must
        /// gather before any update takes place.
        double trans_thresh;

        /// the ammout of rotation that the system must
        /// gather before any update takes place.
        double rot_thresh;

        /// maximum distance (in meters) of the euclidean distance map.
        double l2_max;

        /// resolutions of the maps.
        double resolution;

        /// The side size of a patch
        uint32_t patch_size;

        /// maximum number of iterations that the optimizer
        /// can achieve.
        uint32_t max_iter;

        /// strategy to use in the optimization.
        std::string strategy;

        /// wether or not to keep an execution summary.
        bool keep_summary;
    };

public:

    sdm::SimpleOccupancyMap* occupancy_map;
    sdm::DynamicDistanceMap* distance_map;

    Loc2D() = default;
    void Init(const Options& options = Options());

    virtual ~Loc2D();

    bool enoughMotion(const rtk::geom::Pose2D& odometry);

    bool update(const PointCloudXYZ::Ptr& surface,
                const rtk::geom::Pose2D& odometry,
                double timestamp);

    void triggerGlobalLocalization();

    const Summary* getSummary() const;

    inline void setPose(const geom::Pose2D& pose)
    { pose_ = pose; has_first_scan = false; }

    inline const geom::Pose2D& getPose() const
    { return pose_; }

private:

    void globalLocalization(const PointCloudXYZ::Ptr& surface);

    StrategyPtr makeStrategy(const std::string& name, const VectorXd& parameters);
    RobustCostPtr makeRobust(const std::string& name, const double& param);

private:
    SolverOptions solver_options_;
    Summary* summary_;

    rtk::geom::Pose2D odom_;
    rtk::geom::Pose2D pose_;

    double trans_thresh_;
    double rot_thresh_;
    bool has_first_scan;
    bool do_global_loclization_;
};

}} /* rtk::slam */


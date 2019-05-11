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
#include <rtk/sdm/frequency_occupancy_map.h>

#include "rtk/sdm/export.h"

namespace rtk  {
namespace slam {

class Slam2D {
public:

    typedef rtk::nlls::Solver::Options SolverOptions;
    typedef rtk::nlls::Solver::Summary SolverSummary;

    typedef rtk::math::Stats StatsAccumulator;

    typedef rtk::nlls::Strategy::Ptr   StrategyPtr;
    typedef rtk::nlls::RobustCost::Ptr RobustCostPtr;

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

        /// Execution times for mapping.
        std::vector<double> time_mapping;

        /// Execution times for point cloud transformation
        std::vector<double> time_pc_tf;

        /// Execution times for occupancy map update
        std::vector<double> time_occ;

        /// Execution times for distance map update
        std::vector<double> time_dm;

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

        double truncated_ray;

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

        /// online data compression
        bool use_compression;

        /// size of LRU
        uint32_t cache_size;

        /// compression algorithm to use when compression is activated
        std::string calgorithm;
    };

public:

    Slam2D(const Options& options = Options());

    virtual ~Slam2D();

    bool enoughMotion(const rtk::geom::Pose2D& odometry);

    bool update(const PointCloudXYZ::Ptr& surface,
                const rtk::geom::Pose2D& odometry,
                double timestamp);

    const Summary* getSummary() const;
    uint64_t getMemoryUsage() const;
    uint64_t getMemoryUsage(uint64_t& occmem, uint64_t& dmmem) const;

    uint32_t getNumberOfProcessedCells() const
    { return number_of_proccessed_cells_; }


    void useCompression(bool compression, const std::string& algorithm = "lz4");

    inline void setPose(const geom::Pose2D& pose)
    { pose_ = pose; }

    inline geom::Pose2D getPose() const
    { return pose_; }

    const sdm::FrequencyOccupancyMap* getOccupancyMap() const
    { return occupancy_map_; }

    const sdm::DynamicDistanceMap* getDistanceMap() const
    { return distance_map_; }

    inline void saveOccImage(const std::string& name) const
    { export_to_png(*occupancy_map_, name); }

    inline void saveDistImage(const std::string& name) const
    { export_to_png(*distance_map_, name); }

private:

    StrategyPtr makeStrategy(const std::string& name, const VectorXd& parameters);
    RobustCostPtr makeRobust(const std::string& name, const double& param);

    void updateMaps(const PointCloudXYZ::Ptr& cloud);

private:
    SolverOptions solver_options_;

    sdm::DynamicDistanceMap*    distance_map_;
    sdm::FrequencyOccupancyMap* occupancy_map_;

    Summary* summary_;

    rtk::geom::Pose2D odom_;
    rtk::geom::Pose2D pose_;

    double trans_thresh_;
    double rot_thresh_;
    bool has_first_scan;

    uint32_t number_of_proccessed_cells_;
    double truncated_ray_;
};

}} /* rtk::slam */


//
// Copyright (c) 2018, IRIS Laboratory, IEETA, University of Aveiro - Portugal
//
// @date    2018-10-30
// @authors Eurico Pedrosa <efp@ua.pt>
//

#pragma once

#include <vector>

#include <rtk/math/stats.h>
#include <rtk/geom/pose2d.h>

#include <rtk/nlls/solver.h>

#include <rtk/sdm/dynamic_distance_map.h>
#include <rtk/sdm/frequency_occupancy_map.h>

namespace rtk  {

struct ThreadPool;

namespace slam {

class PFSlam2D {
public:

    typedef rtk::nlls::Solver::Options SolverOptions;
    typedef rtk::nlls::Solver::Summary SolverSummary;

    typedef rtk::nlls::Strategy::Ptr   StrategyPtr;
    typedef rtk::nlls::RobustCost::Ptr RobustCostPtr;

    typedef boost::shared_ptr<sdm::DynamicDistanceMap>    DynamicDistanceMapPtr;
    typedef boost::shared_ptr<sdm::FrequencyOccupancyMap> FrequencyOccupancyMapPtr;


public:

    struct Particle {

        // The weight of the particle
        double weight;

        double normalized_weight;

        double weight_sum;

        // The pose of this particle in the map
        geom::Pose2D pose;

        // history
        std::deque<geom::Pose2D> poses;

        DynamicDistanceMapPtr    dm;
        FrequencyOccupancyMapPtr occ;

    };

    struct Options {
        Options();

        /// the number of particles to use
        uint32_t particles;

        ///
        bool use_gaussian_proposal;

        double srr;
        double str;
        double stt;
        double srt;

        double meas_sigma;
        double meas_sigma_gain;

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

        int32_t threads;

        uint32_t seed;

        ///
        bool use_compression;
        uint32_t cache_size;

        /// compression algorithm to use when compression is activated
        std::string calgorithm;

    };

    struct Summary {
        Summary(){}

        std::string fullReport() const;

        bool saveMetrics(const std::string& filename) const;

        ///
        std::vector<double> timestamps;

        /// Update execution time
        std::vector<double> time_update;

        /// Execution times for scan matching.
        std::vector<double> time_sm;

        /// Execution times for mapping.
        std::vector<double> time_map;

        /// Execution times for resample.
        std::vector<double> time_resample;

        std::vector<double> resample_stamp;

        /// NEFF value
        std::vector<double> neff;

        std::vector<size_t> memory;
    };

public:

    PFSlam2D(const Options& options = Options());

    virtual ~PFSlam2D();

    inline void setSummary(Summary* summary)
    {
        summary_ = summary;
    }

    inline const Options& getOptions() const
    {
        return options_;
    }

    uint64_t getMemoryUsage() const;
    uint64_t getMemoryUsage(uint64_t& occmem, uint64_t& dmmem) const;

    bool update(const PointCloudXYZ::Ptr& surface,
                const rtk::geom::Pose2D& odometry,
                double timestamp);

    size_t getBestParticleIdx() const;

    geom::Pose2D getPose() const;

    inline const std::deque<double>& getTimestamps() const
    { return timestamps_; }

    inline const std::vector<Particle>& getParticles() const
    { return particles_[current_particle_set_]; }

    const sdm::FrequencyOccupancyMap* getOccupancyMap() const
    {
        size_t pidx = getBestParticleIdx();
        return particles_[current_particle_set_][pidx].occ.get();
    }

    const sdm::DynamicDistanceMap* getDistanceMap() const
    {
        size_t pidx = getBestParticleIdx();
        return particles_[current_particle_set_][pidx].dm.get();
    }

    void saveOccImage(const std::string& name) const;

    inline double getNeff() const
    { return neff_; }

    void setPrior(const geom::Pose2D& prior);

private:

    StrategyPtr makeStrategy(const std::string& name, const VectorXd& parameters);
    RobustCostPtr makeRobust(const std::string& name, const double& param);

    void drawFromMotion(const geom::Pose2D& delta, const geom::Pose2D& old_pose, geom::Pose2D& pose);

    double likelihood(const PointCloudXYZ::Ptr& surface, geom::Pose2D& pose);

    double calculateLikelihood(const PointCloudXYZ::Ptr& surface, const geom::Pose2D& pose);
    double calculateLikelihood(const Particle& particle);

    void scanMatch(Particle* particle);
    void updateParticleMaps(Particle* particle);

    void normalize();
    void resample();

private:
    Options options_;
    SolverOptions solver_options_;

    std::vector<Particle> particles_[2];
    uint8_t current_particle_set_;

    rtk::geom::Pose2D odom_;
    rtk::geom::Pose2D pose_;

    double acc_trans_;
    double acc_rot_;
    bool   has_first_scan;

    double truncated_ray_;
    double max_weight_;
    double delta_free_;
    double neff_;

    std::deque<double> timestamps_;
    PointCloudXYZ::Ptr current_surface_;

    Summary* summary_;
    ThreadPool* thread_pool_;
};

}} /* rtk::slam */


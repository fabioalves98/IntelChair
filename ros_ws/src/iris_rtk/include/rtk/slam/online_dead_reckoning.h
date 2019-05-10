//
// Copyright (c) 2018, IRIS Laboratory, IEETA, University of Aveiro - Portugal
//
// @date    2018-10-30
// @authors Eurico Pedrosa <efp@ua.pt>
//

#pragma once

#include <rtk/geom/pose2d.h>

#include <rtk/gis/dynamic_distance_map.h>

namespace rtk  {
namespace slam {

class OnlineDeadReckoning {
public:

    typedef rtk::nlls::Solver::Options SolverOptions;

    typedef rtk::nlls::Strategy::Ptr   StrategyPtr;
    typedef rtk::nlls::RobustCost::Ptr RobustCostPtr;

    typedef boost::shared_ptr<gis::DynamicDistanceMap>    DynamicDistanceMapPtr;

public:

    OnlineDeadReckoning();

    bool update(const PointCloudXYZ::Ptr& surface,
                const rtk::geom::Pose2D& odometry,
                double timestamp);

private:

    void updateMap(const PointCloudXYZ::Ptr& surface);

private:

    SolverOptions solver_options_;
    DynamicDistanceMapPtr distance_map_;

    geom::Pose2D pose_;
    geom::Pose2D odom_;
    bool has_first_scan_;

};

} /* rtk::slam */


//
// Copyright (c) 2018, IRIS Laboratory, IEETA, University of Aveiro - Portugal
//
// @date    2018-10-30
// @authors Eurico Pedrosa <efp@ua.pt>
//

#pragma once

#include <rtk/types.h>
#include <rtk/geom/pose3d.h>

namespace rtk {

/// Single scan from a planar laser range-finder.
struct LaserScan {

    double stamp;               ///< Acquisition time stamp [s]

    geom::Pose3D origin;        ///< pose of the sensor in the robot
    geom::Pose3D location;      ///< pose of the sensor in the world

    double angle_min;           ///< start angle of the scan [rad]
    double angle_max;           ///< end angle of the scan [rad]
    double angle_inc;           ///< angular distance between measurements [rad]

    double range_min;           ///< minimum range value [m]
    double range_max;           ///< maximum range value [m]

    std::vector<double> ranges; ///< range data [m]

    /// Create a point cloud from the laser scan.
    PointCloudXYZ::Ptr toPointCloud()
    {
        const size_t num_ranges = ranges.size();

        PointCloudXYZ::Ptr cloud(new PointCloudXYZ);
        //cloud->header.stamp = stamp * 1000;
        //cloud->is_dense = true;
        //cloud->height   = 0;
        //cloud->width    = num_ranges;

        cloud->sensor_origin_ = origin.xyz();
        cloud->sensor_orientation_.setIdentity();
        /* = origin.se3().so3().unit_quaternion().cast<float>(); */

        double angle = angle_min;
        cloud->points.reserve(num_ranges);
        for (size_t i = 0; i < num_ranges; ++i){

            if (ranges[i] < range_max){

                Vector3d p;

                p.x() = cos(angle) * ranges[i];
                p.y() = sin(angle) * ranges[i];
                p.z() = 0.0;

                cloud->points.push_back(p);
            }

            angle += angle_inc;
        }

        return cloud;
    }
};

} /* rtk */


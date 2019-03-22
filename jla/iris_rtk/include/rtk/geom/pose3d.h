//
// Copyright (c) 2018, IRIS Laboratory, IEETA, University of Aveiro - Portugal
//
// @date    2018-10-30
// @authors Eurico Pedrosa <efp@ua.pt>
//

#pragma once

#include <Eigen/Geometry>

#include <rtk/types.h>
#include <rtk/geom/lie.h>

namespace rtk  {
namespace geom {

class Pose3D {
public:

    Pose3D();
    Pose3D(const double& x, const double& y, const double& z,
           const double& roll, const double& pitch, const double& yaw);
    Pose3D(const Vector3d& xyz, const Vector3d& rpy);
    Pose3D(const Vector3d& xyz, const double& yaw);
    Pose3D(const Matrix4d& transformation);
    Pose3D(const Affine3d& transformation);
    Pose3D(const Pose3D& other);
    Pose3D(const SE3d& se3);

    virtual ~Pose3D();

    Pose3D operator+(const Pose3D& other);
    Pose3D operator-(const Pose3D& other);

    Vector3d operator*(const Vector3d& point);

    Pose3D& operator+=(const Pose3D& other);
    Pose3D& operator-=(const Pose3D& other);

    double x() const;
    double y() const;
    double z() const;
    Vector3d xyz() const;

    double roll() const;
    double pitch() const;
    double yaw() const;
    Vector3d rpy() const;

    inline const SE3d& se3() const
    { return state_; }

    inline SE3d& se3()
    { return state_; }

private:
    SE3d state_;
};

}} /* rtk::geom */


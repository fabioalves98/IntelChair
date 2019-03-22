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

class Pose2D {
public:

    Pose2D();
    Pose2D(const double& x, const double& y, const double& rotation);
    Pose2D(const Vector2d& xy, const double& rotation);
    Pose2D(const Vector3d& xyr);
    Pose2D(const Affine2d& transformation);
    Pose2D(const Pose2D& other);
    Pose2D(const SE2d& se2);

    virtual ~Pose2D();

    Pose2D operator+(const Pose2D& other);
    Pose2D operator-(const Pose2D& other);

    Vector2d operator*(const Vector2d& point);

    Pose2D& operator+=(const Pose2D& other);
    Pose2D& operator-=(const Pose2D& other);

    Pose2D& operator=(const Pose2D& other);

    double x() const;
    double y() const;
    Vector2d xy() const;

    double rotation() const;

    const Vector3d xyr() const;

    inline const SE2d& se2() const
    { return state_; }

    inline SE2d& se2()
    { return state_; }

private:
    SE2d state_;
};

}} /* rtk::geom */


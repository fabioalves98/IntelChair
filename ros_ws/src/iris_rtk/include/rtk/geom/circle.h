//
// Copyright (c) 2018, IRIS Laboratory, IEETA, University of Aveiro - Portugal
//
// @date    2018-10-30
// @authors Eurico Pedrosa <efp@ua.pt>
//

#pragma once

#include <rtk/types.h>

namespace rtk  {
namespace geom {

class Circle {
public:

    Circle(const Vector3d& center, const double& radius);
    Circle(const Circle& other);

    bool isInside(const Vector3d& point);
    double distance(const Vector3d& point);

    inline Vector3d getCenter() const
    { return center_; }

    inline double getRadius() const
    { return radius_; }

public:

    static void buildDiscrete(const Vector3ui& center, uint32_t radius, VectorVector3ui& sink);

private:
    Vector3d center_;
    double   radius_;
};

}} /* rtk::geom */


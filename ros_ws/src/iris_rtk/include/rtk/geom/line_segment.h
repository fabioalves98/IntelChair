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

class LineSegment {
public:

    LineSegment(const Vector3d& point0, const Vector3d& point1);
    LineSegment(const LineSegment& other);

    double distance(const Vector3d& point) const;

    Vector3d nearest(const Vector3d& point) const;

public:

    static void buildDiscrete(const Vector3ui& from, const Vector3ui& to, VectorVector3ui& sink);

private:
    Vector3d point0_;
    Vector3d point1_;
};

}} /* rtk::geom */


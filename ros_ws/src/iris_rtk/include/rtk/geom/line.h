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

class Line {
public:

    Line(const Vector3d& point, const Vector3d direction);
    Line(const Line& other);

    double distance(const Vector3d& point) const;
    double distance(const Line& other) const;

    Vector3d nearest(const Vector3d& point) const;
    Vector3d nearest(const Line& other) const;

public:

    static Line from2Points(const Vector3d& point1, const Vector3d& point2);

private:
    Vector3d point_;
    Vector3d dir_;
};

}} /* rtk::geom */


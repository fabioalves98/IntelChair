//
// Copyright (c) 2018, IRIS Laboratory, IEETA, University of Aveiro - Portugal
//
// @date    2018-10-30
// @authors Eurico Pedrosa <efp@ua.pt>
//

#include <iostream>

#include <rtk/geom/line.h>

namespace rtk  {
namespace geom {

Line::Line(const Vector3d& point, const Vector3d direction)
    : point_(point), dir_(direction.normalized())
{}

Line::Line(const Line& other)
{
    point_ = other.point_;
    dir_   = other.dir_;
}

double Line::distance(const Vector3d& point) const
{
    Vector3d w = point - point_;

    return (w - w.dot(dir_)*dir_).norm();
}

double Line::distance(const Line& other) const
{
    // http://geomalgorithms.com/a07-_distance.html
    Vector3d w = point_ - other.point_;

    double a = dir_.dot(dir_);
    double b = dir_.dot(other.dir_);
    double c = other.dir_.dot(other.dir_);
    double d = dir_.dot(w);
    double e = other.dir_.dot(w);

    double den = a*c - b*b;

    if (fabs(den) < 1e-4)
        return (w + other.dir_ * (e/c)).norm();

    return (w + ((b*e - c*d)*dir_ - (a*e - b*d)*other.dir_)/den).norm();
}

Vector3d Line::nearest(const Vector3d& point) const
{
    Vector3d w = point - point_;

    return dir_.cross(w);
}

Vector3d Line::nearest(const Line& other) const
{
    // http://geomalgorithms.com/a07-_distance.html
    Vector3d w = point_ - other.point_;

    double a = dir_.dot(dir_);
    double b = dir_.dot(other.dir_);
    double c = other.dir_.dot(other.dir_);

    double den = a*c - b*b;

    if (fabs(den) < 1e-4)
        return point_;

    double d = dir_.dot(w);
    double e = other.dir_.dot(w);

    return  point_ + dir_ * (b*e - c*d) / den;
}

//==================================================================================================

Line Line::from2Points(const Vector3d& point1, const Vector3d& point2)
{
    return Line(point1, point2 - point1);
}

}} /* rtk::geom */

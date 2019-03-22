//
// Copyright (c) 2018, IRIS Laboratory, IEETA, University of Aveiro - Portugal
//
// @date    2018-10-30
// @authors Eurico Pedrosa <efp@ua.pt>
//

#include <rtk/geom/line_segment.h>

namespace rtk  {
namespace geom {


LineSegment::LineSegment(const Vector3d& point0, const Vector3d& point1)
    : point0_(point0), point1_(point1)
{}

LineSegment::LineSegment(const LineSegment& other)
{
    point0_ = other.point0_;
    point1_ = other.point1_;
}

double LineSegment::distance(const Vector3d& point) const
{
    Vector3d v = point1_ - point0_;
    Vector3d w = point - point0_;

    double c1 = w.dot(v);
    if (c1 <= 0) // before point0
        return w.norm();

    double c2 = v.dot(v);
    if (c2 <= c1) // after point1
        return (point - point1_).norm();

    return (point - (point0_ + v * c1 / c2)).norm();
}

Vector3d LineSegment::nearest(const Vector3d& point) const
{
    Vector3d v = point1_ - point0_;
    Vector3d w = point - point0_;

    double c1 = w.dot(v);
    if (c1 <= 0) // before point0
        return point0_;

    double c2 = v.dot(v);
    if (c2 <= c1) // after point1
        return point1_;

    return (point0_ + v * c1 / c2);
}

//==================================================================================================

void LineSegment::buildDiscrete(const Vector3ui& from, const Vector3ui& to, VectorVector3ui& sink)
{
    if ( from == to ) return;

    Vector3l error = Vector3l::Zero();
    Vector3l coord = from.cast<int64_t>();
    Vector3l delta = to.cast<int64_t>() - coord;

    Vector3l step = (delta.array() < 0).select(-1, Vector3l::Ones());

    delta = delta.array().abs();

    // maximum change of any coordinate
    const int n = delta.maxCoeff();
    for (int i = 0; i < n - 1; ++i){
        // update errors
        error += delta;

        for (int j = 0; j < 3; ++j)
            if ( (error(j) << 1) >= n ){
                coord(j) += step(j);
                error(j) -= n;
            }

        // save the coordinate
        sink.push_back(coord.cast<uint32_t>() );
    }
}

}} /* rtk::geom */

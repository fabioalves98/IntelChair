//
// Copyright (c) 2018, IRIS Laboratory, IEETA, University of Aveiro - Portugal
//
// @date    2018-10-30
// @authors Eurico Pedrosa <efp@ua.pt>
//

#include <rtk/geom/circle.h>

namespace rtk  {
namespace geom {

Circle::Circle(const Vector3d& center, const double& radius)
    : center_(center),
      radius_(radius)
{}

Circle::Circle(const Circle& other)
{
    center_ = other.center_;
    radius_ = other.radius_;
}

bool Circle::isInside(const Vector3d& point)
{
    return (center_ - point).squaredNorm() < radius_ * radius_;
}

double Circle::distance(const Vector3d& point)
{
    return (center_ - point).norm();
}

//==================================================================================================

void Circle::buildDiscrete(const Vector3ui& center, uint32_t radius, VectorVector3ui& sink)
{
    if (radius == 0)
        return;

    int64_t x0 = center.x();
    int64_t y0 = center.y();

    int64_t x = -radius;
    int64_t y = 0;
    int64_t err = 2 - 2*radius;
    int64_t perr;

    do {
        sink.push_back(Vector3l(x0-x, y0+y).cast<uint32_t>());
        sink.push_back(Vector3l(x0-y, y0-x).cast<uint32_t>());
        sink.push_back(Vector3l(x0+x, y0-y).cast<uint32_t>());
        sink.push_back(Vector3l(x0+x, y0+x).cast<uint32_t>());

        perr = err;
        if (perr <= y) err += ++y*2+1;
        if (perr > x || err > y) err += ++x*2+1;
    } while(x < 0);

}

}} /* rtk::geom */

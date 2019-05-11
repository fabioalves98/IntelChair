//
// Copyright (c) 2018, IRIS Laboratory, IEETA, University of Aveiro - Portugal
//
// @date    2018-10-30
// @authors Eurico Pedrosa <efp@ua.pt>
//

#include <rtk/geom/pinhole.h>

namespace rtk  {
namespace geom {

Pinhole::Pinhole(int32_t width, int32_t height, Matrix3d K)
{
    width_  = width;
    height_ = height;

    cx_ = K(0,2);
    cy_ = K(1,2);

    fx_ = K(0,0);
    fy_ = K(1,1);

    fx_inv_ = 1.0 / fx_;
    fy_inv_ = 1.0 / fy_;
}

Pinhole::Pinhole(int32_t width, int32_t height,
                double cx, double cy, double fx, double fy)
    : width_(width), height_(height),
      cx_(cx), cy_(cy), fx_(fx), fy_(fy),
      fx_inv_(1.0/fx), fy_inv_(1.0/fy)
{ }

}} /* rtk::geom  */

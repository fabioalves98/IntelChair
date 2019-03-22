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


class Pinhole {
public:

    Pinhole(int32_t width, int32_t height, Matrix3d K);
    Pinhole(int32_t with, int32_t height, double cx, double cy, double fx, double fy);

    /**
     * Project from pixels to world coordinates.
     */
    inline Vector3d c2w(const double& u, const double& v) const
    {
        Eigen::Vector3d xyz;

        xyz[0] = (u - cx_) * fx_inv_;
        xyz[1] = (v - cy_) * fy_inv_;
        xyz[2] = 1.0;

        return xyz;
    }


private:

    int32_t width_;
    int32_t height_;

    double cx_;
    double cy_;

    double fx_;
    double fy_;

    double fx_inv_;
    double fy_inv_;
};


}} /* rtk::geom  */



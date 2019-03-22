//
// Copyright (c) 2018, IRIS Laboratory, IEETA, University of Aveiro - Portugal
//
// @date    2018-10-30
// @authors Eurico Pedrosa <efp@ua.pt>
//

#pragma once

#include "rtk/_vendor/sophus/se2.hpp"
#include "rtk/_vendor/sophus/se3.hpp"

namespace rtk  {
namespace geom {

typedef Sophus::SO2d SO2d;
typedef Sophus::SO3d SO3d;

typedef Sophus::SE2d SE2d;
typedef Sophus::SE3d SE3d;

}} /* rtk::geometry */


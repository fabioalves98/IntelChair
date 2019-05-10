//
// Copyright (c) 2018, IRIS Laboratory, IEETA, University of Aveiro - Portugal
//
// @date    2018-10-30
// @authors Eurico Pedrosa <efp@ua.pt>
//

#pragma once

#include <rtk/types.h>

namespace rtk  {
namespace math {

namespace random {

uint32_t genSeed();
void setSeed(uint32_t seed);

double  uniform();
int32_t uniform(int32_t from, int32_t to);

double normal(double stddev = 1.0);

} /* random  */

}} /* rtk::math */


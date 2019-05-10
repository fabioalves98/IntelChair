//
// Copyright (c) 2018, IRIS Laboratory, IEETA, University of Aveiro - Portugal
//
// @date    2018-10-30
// @authors Eurico Pedrosa <efp@ua.pt>
//

#include <random>

#include <boost/random/random_device.hpp>
#include <boost/random/mersenne_twister.hpp>

#include <boost/random/uniform_real_distribution.hpp>
#include <boost/random/uniform_int_distribution.hpp>
#include <boost/random/normal_distribution.hpp>

#include <rtk/math/random.h>

static std::random_device rnd;
static std::mt19937       gen(rnd());

namespace rtk  {
namespace math {

namespace random {

uint32_t genSeed()
{
    return rnd();
}

void setSeed(uint32_t seed)
{
    gen.seed(seed);
}

double uniform()
{
    std::uniform_real_distribution<double> distribution(0.0, 1.0);
    return distribution(gen);
}

double uniform(double low, double high)
{
    std::uniform_real_distribution<double> distribution(low, high);
    return distribution(gen);
}

int32_t uniform(int32_t from, int32_t to)
{
    std::uniform_int_distribution<int32_t> distribution(from, to);
    return distribution(gen);
}

double normal(double stddev)
{
    std::normal_distribution<double> distribution(0.0, stddev);
    return distribution(gen);
}

} /* random  */

}} /* rtk::math */

//
// Copyright (c) 2018, IRIS Laboratory, IEETA, University of Aveiro - Portugal
//
// @date    2018-10-30
// @authors Eurico Pedrosa <efp@ua.pt>
//

#include <cmath>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/min.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/variance.hpp>

#include <rtk/math/stats.h>

using namespace boost;

namespace rtk  {
namespace math {

Stats::Stats()
{
    acc = new acc_t;
}

Stats::~Stats()
{
    delete acc;
}

void Stats::operator()(const double& sample)
{
    (*acc)(sample);
}

void Stats::operator()(const std::vector<double>& samples)
{
    const size_t num_samples = samples.size();
    for (size_t i = 0; i < num_samples; ++i)
        (*acc)(samples[i]);
}

double Stats::min() const
{
    return accumulators::min(*acc);
}

double Stats::max() const
{
    return accumulators::max(*acc);
}

double Stats::mean() const
{
    return accumulators::mean(*acc);
}

double Stats::var() const
{
    return accumulators::variance(*acc);
}

double Stats::std() const
{
    return std::sqrt(var());
}

double Stats::median() const
{
    return accumulators::median(*acc);
}

double Stats::sum() const
{
    return accumulators::sum(*acc);
}

}} /* rtk::math */

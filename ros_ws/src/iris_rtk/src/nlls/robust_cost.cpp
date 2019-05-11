//
// Copyright (c) 2018, IRIS Laboratory, IEETA, University of Aveiro - Portugal
//
// @date    2018-10-30
// @authors Eurico Pedrosa <efp@ua.pt>
//

#include "rtk/nlls/robust_cost.h"

namespace rtk  {
namespace nlls {

double UnitWeight::value(const double& x)
{
    return 1.0;
}

TukeyWeight::TukeyWeight(const double& b)
    : bb_(b*b)
{ }

double TukeyWeight::value(const double& x)
{
    const double xx = x*x;
    if ( xx <= bb_ ){
        const double w = 1.0 - xx / bb_;

        return w*w;
    }

    return 0.0;
}

TDistributionWeight::TDistributionWeight(const double& dof)
    : dof_(dof)
{}

double TDistributionWeight::value(const double& x)
{
    return ((dof_ + 1.0f) / (dof_ + (x * x)));
}

CauchyWeight::CauchyWeight(const double& param)
    : c_(1.0 / (param*param))
{}

double CauchyWeight::value(const double& x)
{
    return (1.0 / (1.0 + x*x * c_));
}

}} /* tk::nlls */

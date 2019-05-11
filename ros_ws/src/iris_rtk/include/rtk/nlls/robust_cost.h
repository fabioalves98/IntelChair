//
// Copyright (c) 2018, IRIS Laboratory, IEETA, University of Aveiro - Portugal
//
// @date    2018-10-30
// @authors Eurico Pedrosa <efp@ua.pt>
//

#pragma once

#include <boost/shared_ptr.hpp>
#include <rtk/types.h>

namespace rtk  {
namespace nlls {

class RobustCost {
public:
    virtual double value(const double& x) = 0;
public:
    typedef boost::shared_ptr<RobustCost> Ptr;
};

class UnitWeight : public RobustCost {
public:
    double value(const double& x);
};

class TukeyWeight : public RobustCost {
public:

    TukeyWeight(const double& b = 4.6851f);
    double value(const double& x);

private:
    double bb_;

};

class TDistributionWeight : public RobustCost {
public:

    TDistributionWeight(const double& dof);
    double value(const double& x);

private:
    double dof_;
};

class CauchyWeight : public RobustCost {
public:

    CauchyWeight(const double& param);
    double value(const double& x);

private:
    double c_;
};

}} /* tk::nlls */



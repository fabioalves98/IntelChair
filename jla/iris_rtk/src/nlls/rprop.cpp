//
// Copyright (c) 2018, IRIS Laboratory, IEETA, University of Aveiro - Portugal
//
// @date    2018-10-30
// @authors Eurico Pedrosa <efp@ua.pt>
//

#include <cmath>
#include <Eigen/Cholesky>

#include <rtk/time/timer.h>
#include <rtk/nlls/rprop.h>

namespace rtk  {
namespace nlls {

inline double s(double value)
{
    if(value>0.0) return  1.0;
    if(value<0.0) return -1.0;

    return 0.0;
}

RProp::Options::Options()
{
    eps1 = 1e-4;
    eps2 = 1e-3;
}

RProp::RProp(const Options& options)
    : opt_(options)
{}

void RProp::reset()
{
    stop_   = false;
    delta_  = VectorXd();
}

VectorXd RProp::step(const VectorXd& residuals, const MatrixXd& J, Strategy::Summary& summary)
{
    time::Timer timer(true);

    chi2_ = residuals.squaredNorm();
    //-------------------------
    summary.cost = chi2_;
    //-------------------------

    VectorXd curgrad = J.transpose() * residuals;

    if ( curgrad.lpNorm<Eigen::Infinity>() <= opt_.eps1){
        stop_ = true;
        return VectorXd::Zero(J.cols());
    }

    if (delta_.size() == 0){
        delta_ = VectorXd::Ones(J.cols()) * 0.01;
        grad_  = VectorXd::Zero(J.cols());
    }

    grad_ = grad_.cwiseProduct(curgrad);

    //------------------------------------------------
    summary.max_abs_g = curgrad.lpNorm<Eigen::Infinity>();
    summary.max_abs_h = 0.0;
    //------------------------------------------------
    if (summary.max_abs_g < opt_.eps1){
        stop_ = true;
        //-----------------------------------------------
        summary.execution_time = timer.elapsed().toSec();
        //-----------------------------------------------
        return VectorXd::Zero(J.cols());
    }

    const size_t n = J.cols();
    VectorXd h(n);
    for (size_t i = 0; i < n; ++i){

        if (grad_[i] > 0.0){
            delta_[i] = std::min(delta_[i] * 1.2, 2.0);
        }
        else if (grad_[i] < 0.0)
        {
            delta_[i] = std::max(delta_[i] * 0.5, 0.0);
        }

        h[i] = - s(curgrad[i]) * delta_[i];
    }

    if ( delta_.lpNorm<Eigen::Infinity>() <= opt_.eps2){
        stop_ = true;
        return VectorXd::Zero(J.cols());
    }

    grad_ = curgrad;

    //------------------------------------------------
    summary.max_abs_h = h.lpNorm<Eigen::Infinity>();
    //------------------------------------------------
    if (summary.max_abs_h < opt_.eps2)
        stop_ = true;

    //-----------------------------------------------
    summary.execution_time = timer.elapsed().toSec();
    //-----------------------------------------------
    return h;
}

bool RProp::valid(const VectorXd& residuals)
{
    if (stop_) return true;

    /* double dF  = chi2_ - residuals.squaredNorm(); */
    /* if (dF > 0){ */
        return true;
    /* } */

    stop_ = true;
    return false;
}

bool RProp::stop()
{
    return stop_;
}

}} /* tk::nlls */

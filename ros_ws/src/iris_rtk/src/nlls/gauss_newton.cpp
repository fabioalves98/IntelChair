//
// Copyright (c) 2018, IRIS Laboratory, IEETA, University of Aveiro - Portugal
//
// @date    2018-10-30
// @authors Eurico Pedrosa <efp@ua.pt>
//

#include <Eigen/Cholesky>

#include "rtk/time/timer.h"
#include "rtk/nlls/gauss_newton.h"

namespace rtk  {
namespace nlls {

GaussNewton::Options::Options()
{
    eps1 = 1e-4;
    eps2 = 1e-4;
}

GaussNewton::GaussNewton(const Options& options)
    : opt_(options)
{}

void GaussNewton::reset()
{
    stop_ = false;
}

VectorXd GaussNewton::step(const VectorXd& residuals, const MatrixXd& J, Strategy::Summary& summary)
{
    time::Timer timer(true);

    //------------------------------
    chi2_ = residuals.squaredNorm();
    summary.cost = chi2_;
    //------------------------------

    VectorXd g = J.transpose() * residuals;
    //------------------------------------------------
    summary.max_abs_g = g.lpNorm<Eigen::Infinity>();
    summary.max_abs_h = 0.0;
    //------------------------------------------------
    if (summary.max_abs_g < opt_.eps1){
        stop_ = true;
        //-----------------------------------------------
        summary.execution_time = timer.elapsed().toSec();
        //-----------------------------------------------
        return VectorXd::Zero(J.cols());
    }

    MatrixXd A = J.transpose() * J;
    // Solve the system
    VectorXd h = A.selfadjointView<Eigen::Lower>().ldlt().solve(-g);
    //------------------------------------------------
    summary.max_abs_h = h.lpNorm<Eigen::Infinity>();
    //------------------------------------------------
    if (summary.max_abs_h < opt_.eps2)
        stop_ = true;

    //-----------------------------------------------
    summary.trust_radius   = 1;
    summary.execution_time = timer.elapsed().toSec();
    //-----------------------------------------------
    return h;
}

bool GaussNewton::valid(const VectorXd& residuals)
{
    if (stop_) return true;

    double dF  = chi2_ - residuals.squaredNorm();
    if (dF > 0){
        return true;
    }

    stop_ = true;
    return false;
}

bool GaussNewton::stop()
{
    return stop_;
}

}} /* rtk::nlls */

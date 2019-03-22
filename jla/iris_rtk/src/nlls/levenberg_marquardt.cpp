//
// Copyright (c) 2018, IRIS Laboratory, IEETA, University of Aveiro - Portugal
//
// @date    2018-10-30
// @authors Eurico Pedrosa <efp@ua.pt>
//

#include <cmath>
#include <Eigen/Cholesky>

#include <rtk/time/timer.h>
#include <rtk/nlls/levenberg_marquardt.h>

namespace rtk  {
namespace nlls {

LevenbergMarquard::Options::Options()
{
    eps1 = 1e-4;
    eps2 = 1e-4;
    tau  = 1e-4;
}

LevenbergMarquard::LevenbergMarquard(const Options& options)
    : opt_(options)
{}

void LevenbergMarquard::reset()
{
    mu_ = -1;
    v_  = 2.0;
    stop_ = false;
}

VectorXd LevenbergMarquard::step(const VectorXd& residuals, const MatrixXd& J,
                                 Strategy::Summary& summary)
{
    time::Timer timer(true);

    chi2_ = residuals.squaredNorm();
    //-------------------
    summary.cost = chi2_;
    //-------------------

    g_ = J.transpose() * residuals;
    //------------------------------------------------
    summary.max_abs_g = g_.lpNorm<Eigen::Infinity>();
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
    // mu_ is allways positive because the diagonal of A is also allways
    // positive. Therefore, a negative mu_ requires initialization.
    if ( mu_ < 0 )
        mu_ = opt_.tau * A.diagonal().maxCoeff();

    //-------------------------
    summary.trust_radius = mu_;
    //-------------------------

    A.diagonal().array() += (mu_ );
    // Solve the system
    h_ = A.selfadjointView<Eigen::Upper>().llt().solve(-g_);
    //------------------------------------------------
    summary.max_abs_h = h_.lpNorm<Eigen::Infinity>();
    //------------------------------------------------
    if (summary.max_abs_h < opt_.eps2)
        stop_ = true;

    //-----------------------------------------------
    summary.execution_time = timer.elapsed().toSec();
    //-----------------------------------------------
    return h_;
}

bool LevenbergMarquard::valid(const VectorXd& residuals)
{
    if (stop_) return true;

    double dF = (chi2_ - residuals.squaredNorm());
    double dL = 0.5*h_.transpose().dot(mu_*h_ - g_);

    if ( dL > 0.0 and dF > 0.0 ){
        // update mu
        mu_ = mu_ * std::max( 1.0 / 3.0, 1 - std::pow(2*(dF/dL)-1,3) );
        v_  = 2.0;

        return true;
    }

    mu_ = mu_ * v_; v_ = 2*v_;
    return false;
}

bool LevenbergMarquard::stop()
{
    return stop_;
}

}} /* tk::nlls */

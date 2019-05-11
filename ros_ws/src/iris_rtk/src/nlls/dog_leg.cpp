//
// Copyright (c) 2018, IRIS Laboratory, IEETA, University of Aveiro - Portugal
//
// @date    2018-10-30
// @authors Eurico Pedrosa <efp@ua.pt>
//

#include <cmath>
#include <Eigen/Cholesky>

#include <rtk/time/timer.h>
#include <rtk/nlls/dog_leg.h>

namespace rtk  {
namespace nlls {

DogLeg::Options::Options()
{
    eps1 = 1e-4;
    eps2 = 1e-4;
    radius = 1.0;
}

DogLeg::DogLeg(const Options& options)
    : opt_(options)
{}

void DogLeg::reset()
{
    radius_ = opt_.radius;
    solved_ = false;
    stop_   = false;
}

VectorXd DogLeg::step(const VectorXd& residuals, const MatrixXd& J, Strategy::Summary& summary)
{
    time::Timer timer(true);

    chi2_ = residuals.squaredNorm();
    //-------------------------
    summary.cost = chi2_;
    summary.trust_radius = radius_;
    //-------------------------

    VectorXd hsd = -J.transpose() * residuals;
    //------------------------------------------------
    summary.max_abs_g = hsd.lpNorm<Eigen::Infinity>();
    summary.max_abs_h = 0.0;
    //------------------------------------------------
    if (summary.max_abs_g < opt_.eps1){
        stop_ = true;
        //-----------------------------------------------
        summary.execution_time = timer.elapsed().toSec();
        //-----------------------------------------------
        return VectorXd::Zero(J.cols());
    }

    // Calculate  the dog leg step
    // ===========================
    VectorXd hdl;

    // cache some values.
    double hsd_norm  = hsd.norm();
    double hsd_norm2 = hsd.squaredNorm();

    double alpha = hsd_norm2 / (J * hsd).squaredNorm();
    if( alpha * hsd_norm >= radius_ ){
        gain_denominator_ = (radius_ * (2.0*alpha * hsd_norm - radius_)) / (2.0*alpha);
        hdl = (radius_ / hsd_norm) * hsd;
    } else {

        if ( not solved_ ){
            hgn_ = (J.transpose() * J).selfadjointView<Upper>().ldlt().solve(hsd);
            solved_ = true;
        }

        if( hgn_.norm() <= radius_ ){
            gain_denominator_ = 0.5 * chi2_;
            hdl = hgn_;
        } else {

            VectorXd a = alpha * hsd;
            VectorXd b = hgn_;

            double c         = a.dot(b - a);
            double b_a_norm2 = (b - a).squaredNorm();
            double a_norm2   = a.squaredNorm();
            double r2        = radius_ * radius_;
            double sqrt_term = std::sqrt(c*c + b_a_norm2 * (r2 - a_norm2));
            double beta;

            if( c <= 0 )
                beta = (-c + sqrt_term) / (b_a_norm2);
            else
                beta = (r2 - a_norm2) / (c + sqrt_term);

            gain_denominator_ = 0.5 * alpha * (1 - beta) * (1 - beta) *
                hsd_norm2 + beta * (2 - beta) * chi2_;
            hdl = alpha * hsd + beta * (hgn_ - alpha * hsd);
        }
    } // end if ( alpha * ||hsd||^2 )

    h_norm_ = hdl.norm();
    //------------------------------------------------
    summary.max_abs_h = hdl.lpNorm<Eigen::Infinity>();
    //------------------------------------------------
    if (summary.max_abs_h < opt_.eps2)
        stop_ = true;

    //-----------------------------------------------
    summary.execution_time = timer.elapsed().toSec();
    //-----------------------------------------------
    return hdl;
}

bool DogLeg::valid(const VectorXd& residuals)
{
    if (stop_) return true;

    double ur_norm2 = residuals.squaredNorm();
    double dF = 0.5*(chi2_ - ur_norm2);

    if (gain_denominator_ != 0){
        double gain = dF / gain_denominator_;

        if (gain > .75)
            radius_ = std::max(radius_, 3*h_norm_);
        else if (gain < .25)
            radius_ = radius_ * 0.5;
    }

    if ( gain_denominator_ > 0.0 and dF > 0.0 ){
        solved_ = false;
        return true;
    }

    return false;
}

bool DogLeg::stop()
{
    return stop_;
}

}} /* tk::nlls */

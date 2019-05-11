//
// Copyright (c) 2018, IRIS Laboratory, IEETA, University of Aveiro - Portugal
//
// @date    2018-10-30
// @authors Eurico Pedrosa <efp@ua.pt>
//

#include "rtk/print.h"

#include "rtk/time/timer.h"
#include "rtk/math/stats.h"

#include "rtk/nlls/solver.h"
#include "rtk/nlls/gauss_newton.h"

namespace rtk  {
namespace nlls {

Solver::Options::Options()
{
    max_iterations = 100;

    strategy.reset(new GaussNewton);
    robust_cost.reset(new UnitWeight);

    write_to_stdout = false;
}

Solver::Summary::Summary()
{
    num_successful_steps   = 0;
    num_unsuccessful_steps = 0;
}

std::string Solver::Summary::briefReport() const
{
    return format("ok: %d, fail: %d", num_successful_steps, num_unsuccessful_steps);
}

std::string Solver::Summary::fullReport() const
{
    std::string report;

    // Header
    report = format("== Toolkit - NLLS Optimization Report ==\n\n");

    // Table Header
    report += format("iter%*ccost%*ccost_change%*c|gradient|%*c"
                     "|step|%*ctr_radius%*cstep_time%*cpprocessing"
                     "%*citer_time\n",
                     4, ' ', 6, ' ', 3, ' ', 5, ' ', 5, ' ', 4, ' ', 3, ' ', 4, ' ');

    math::Stats exec_time_stats;
    math::Stats exec_time_residual;
    double prev_cost = 0;
    for(int32_t i = 0; i < step_summaries.size(); ++i){
        Strategy::Summary summ = step_summaries[i];

        if (i == 0)
            prev_cost = summ.cost;

        report += format("%3d  %11.6g  %11.6g  %11.6g  %11.6g  %11.6g  %11.6g  %11.6g  "
                          "%11.6g\n",
                          i, summ.cost, prev_cost - summ.cost,
                          summ.max_abs_g, summ.max_abs_h,
                          summ.trust_radius,
                          summ.execution_time,
                          execution_time_preprocessing[i],
                          execution_time_iter[i]);

        prev_cost = summ.cost;
        exec_time_stats(summ.execution_time);

        // execution time residuals
        double etr = execution_time_iter[i] -
                     (summ.execution_time + execution_time_preprocessing[i]);
        exec_time_residual(etr);
    }

    report += format("\nStrategy\n"
                     "  Name       %s\n",
                     strategy_name.c_str());

    report += format("\nCost:\n"
                     "  Initial    %-g\n"
                     "  Final      %-g\n"
                     "  Change     %-g\n",
                     initial_cost, final_cost,
                     initial_cost - final_cost);

    report += format("\nIterations\n"
                     "  Successful          %-d\n"
                     "  Unsuccessful        %-d\n"
                     "  Total               %-d\n",
                     num_successful_steps,
                     num_unsuccessful_steps,
                     num_successful_steps+num_unsuccessful_steps);

    double mean_pp = math::Stats::mean(execution_time_preprocessing);
    double std_pp  = math::Stats::std(execution_time_preprocessing, mean_pp);

    double mean_eti = math::Stats::mean(execution_time_iter);
    double std_eti  = math::Stats::std(execution_time_iter, mean_eti);

    report += format("\nTime per iteration (in miliseconds)\n"
                     "  Residual          %-g +/- %g\n"
                     "  Step              %-g +/- %g\n"
                     "  Pre-processing    %-g +/- %g\n"
                     "  Total             %-g +/- %g\n",
                     exec_time_residual.mean()*1000, exec_time_residual.std()*1000,
                     exec_time_stats.mean()*1000, exec_time_stats.std()*1000,
                     mean_pp*1000, std_pp*1000,
                     mean_eti*1000, std_eti*1000);

    report = format("\nTotal optimization time (in seconds)  %g\n", execution_time);
    return report;
}

//==================================================================================================

Solver::Solver(const Solver::Options& options)
    : options_(options)
{}

void Solver::solve(Problem& problem, Solver::Summary* summary, MatrixXd* cov)
{
    time::Timer time(true);

    VectorXd r;  // Current residuals.
    VectorXd ur; // Updated residuals residuals.
    MatrixXd J;  // Jacobian with respect to the parameters.
    VectorXd h;  // current step

    VectorXd w;

    Strategy::Ptr strategy = options_.strategy;

    //------------------------------------------------
    if (summary){
        problem.eval(r, 0);
        summary->initial_cost = r.squaredNorm() * 0.5;
        summary->strategy_name= strategy->name();
    }//-----------------------------------------------

    strategy->reset();
    bool valid = true;
    uint32_t iter = 0;
    while (not strategy->stop() and iter < options_.max_iterations){
        time::Timer iter_time(true);

        if (valid){
            time::Timer pp_time(true);
            // 1. compute residuals and jacobian
            problem.eval(r, &J);

            // 2. compute and apply weights
            const int32_t rows = r.rows();
            for (int32_t i = 0; i < rows; i++){
                double w = std::sqrt(options_.robust_cost->value(r[i]));
                r[i]    *= w;
                J.row(i) *= w;
            }

            //-----------------------------------------------------------------------------
            if (summary)
                summary->execution_time_preprocessing.push_back(pp_time.elapsed().toSec());
            //-----------------------------------------------------------------------------
        }

        // 3. do optimization step
        Strategy::Summary step_summary;
        h = strategy->step(r, J, step_summary);
        //--------------------------------------------------
        if (summary)
            summary->step_summaries.push_back(step_summary);
        //--------------------------------------------------
        problem.update(h);

        // 4. verify step validity
        time::Timer pp2_time(true);
        problem.eval(ur, 0);

        const int32_t rows = r.rows();
        for (int32_t i = 0; i < rows; i++){
            double w = std::sqrt(options_.robust_cost->value(ur[i]));
            ur[i]    *= w;
        }

        //----------------------------------------------------------------------------------
        if (summary){
            if (valid)
                summary->execution_time_preprocessing.back() += pp2_time.elapsed().toSec();
            else
                summary->execution_time_preprocessing.push_back(pp2_time.elapsed().toSec());
        }//---------------------------------------------------------------------------------

        valid = strategy->valid(ur);
        if (not valid){
            // revert the step
            problem.update(-h);
        }

        // 5. increment iteration count
        ++iter;

        //----------------------------------------
        if (summary){
            summary->execution_time_iter.push_back(iter_time.elapsed().toSec());
            if (valid)
                summary->num_successful_steps++;
            else
                summary->num_unsuccessful_steps++;
        }//---------------------------------------

        if (options_.write_to_stdout){
            if (iter == 1)
                print("iter%*ccost%*c|gradient|%*c"
                      "|step|%*ctr_radius%*cstep_time%*citer_time\n",
                      4, ' ', 6, ' ', 5, ' ', 6, ' ', 5, ' ', 5, ' ');

            print("%3d  %11.6g  %11.6g  %11.6g  %11.6g  %11.6g  %11.6g\n",
                  iter-1, step_summary.cost,
                  step_summary.max_abs_g, step_summary.max_abs_h,
                  step_summary.trust_radius,
                  step_summary.execution_time,
                  iter_time.elapsed().toSec());
        }
    };

    if (cov){
        problem.eval(r, &J);
        w.resize(r.size());
        computeWeights(r, w);
        scaleJacobian(w, J);

        calculateCovariance(J, cov);
    }

    //----------------------------------------------------
    if (summary){
        problem.eval(r, 0);
        summary->final_cost = r.squaredNorm() * 0.5;
        summary->execution_time = time.elapsed().toSec();
    }//---------------------------------------------------
}

void Solver::computeWeights(const VectorXd& residuals, VectorXd& weights)
{
    const int32_t rows = residuals.rows();
    for (int32_t i = 0; i < rows; i++)
        weights[i] = std::sqrt(options_.robust_cost->value(residuals[i]));
}

void Solver::scaleJacobian(const VectorXd& weights, MatrixXd& J)
{
    const int32_t rows = weights.rows();
    for (int32_t i = 0; i < rows; ++i)
        J.row(i) *= weights[i];
}

void Solver::calculateCovariance(const MatrixXd& J, MatrixXd* cov) const
{
    Eigen::ColPivHouseholderQR<MatrixXd> qr(J);

    bool QRok = (qr.info() == Eigen::Success) &&
                (qr.rank() == J.cols());

    if ( QRok ){
        Eigen::MatrixXd R = qr.matrixR().triangularView<Eigen::Upper>();
        *cov = (qr.matrixR().transpose().triangularView<Eigen::Lower>() * R ).inverse();
    }else{
        Eigen::JacobiSVD<MatrixXd> svd(J, Eigen::ComputeThinV);
        Eigen::VectorXd sv = svd.singularValues();

        const double eps=1.e-3; // choose your tolerance wisely!
        sv = (sv.array().abs() > eps).select(sv.array().square().inverse(), 0);
        *cov = svd.matrixV() * sv.asDiagonal() * svd.matrixV().transpose();
    }
}

//==================================================================================================

void Solve(const Solver::Options& options, Problem& problem, Solver::Summary* summary, MatrixXd* cov)
{
    Solver solver(options);
    solver.solve(problem, summary, cov);
}

}} /* tk::nlls  */

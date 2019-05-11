//
// Copyright (c) 2018, IRIS Laboratory, IEETA, University of Aveiro - Portugal
//
// @date    2018-10-30
// @authors Eurico Pedrosa <efp@ua.pt>
//

#pragma once

#include <vector>
#include <string>

#include <rtk/types.h>
#include <rtk/nlls/problem.h>
#include <rtk/nlls/strategy.h>
#include <rtk/nlls/robust_cost.h>

namespace rtk  {
namespace nlls {

/**
 * Class for solving nonlinear least-squares (NLLS) problems.
 */
class Solver {
public:

    struct Options {
        Options();

        /// Maximum number of iterations that can be executed.
        uint32_t max_iterations;

        /// Strategy used by the solver.
        Strategy::Ptr strategy;

        /// Weight functions used by the solver.
        RobustCost::Ptr robust_cost;

        /// The solver will output the summary of each iteration,
        /// as it happens, if this flag is set to true.
        bool write_to_stdout;
    };

    struct Summary {

        Summary();

        /**
         * Brief one line description of the state of the solver after execution.
         */
        std::string briefReport() const;

        std::string fullReport() const;

        std::string lastIterationReport() const;

        /// Reason why the solver terminated.
        std::string message;

        /// Cost of the problem before optimization.
        double initial_cost;

        /// Cost of the problem  after optimization.
        double final_cost;

        /// Number of iterations in which the optimization step
        /// was accepted.
        uint32_t num_successful_steps;

        /// Number of iterations in wich the optimization step
        /// was rejected.
        uint32_t num_unsuccessful_steps;

        /// Holds a summary for each step required by the solver.
        std::vector<Strategy::Summary> step_summaries;

        /// The time it took to execute the optimization.
        double execution_time;

        /// Holds the time it took to calculate residuals and weights.
        std::vector<double> execution_time_preprocessing;

        /// Holds the time it took to execute each iteration.
        std::vector<double> execution_time_iter;

        /// The name of the used strategy.
        std::string strategy_name;
    };

public:

    Solver(const Options& options = Options());

    void solve(Problem& problem, Solver::Summary* summary = 0, MatrixXd* cov = 0);
private:

    void computeWeights(const VectorXd& residuals, VectorXd& weights);
    void scaleJacobian(const VectorXd& weights, MatrixXd& J);
    void calculateCovariance(const MatrixXd& J, MatrixXd* cov) const;

private:
    Options options_;
};

void Solve(const Solver::Options& options, Problem& problem, Solver::Summary* summary = 0, MatrixXd* cov = 0);

}} /* tk::nlls  */


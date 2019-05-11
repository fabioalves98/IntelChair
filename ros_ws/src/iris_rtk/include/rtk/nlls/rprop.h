//
// Copyright (c) 2018, IRIS Laboratory, IEETA, University of Aveiro - Portugal
//
// @date    2018-10-30
// @authors Eurico Pedrosa <efp@ua.pt>
//

#pragma once

#include <rtk/nlls/strategy.h>

namespace rtk  {
namespace nlls {

class RProp : public Strategy {
public:

    struct Options {
        Options();
        /// Threshold of the first termination criterion
        /// An optimization step will terminate when max(abs( J'r )) < eps1
        double eps1;

        /// Threshold of the second termination criterion
        /// An optimization step will terminate when max(abs( h )) < eps2,
        /// where h is the solution for J'J*h = J'r.
        double eps2;

    };

public:

    RProp(const Options& options = Options());

    /**
     * Reset the optimization strategy to its initial state.
     */
    void reset();

    /**
     * Performe an optimization step.
     *
     * @param[in]  residuals The problem residuals.
     * @param[in]  J         The Jacobian matrix.
     * @param[out] summary
     *
     * @returns The optimization step.
     */
    VectorXd step(const VectorXd& residuals, const MatrixXd& J, Summary& summary);

    /**
     * Check the validity of the optimization step.
     *
     * @param[in] residuals The residuals after the optimization step.
     *
     * @returns True if the last step is valid, false otherwise.
     */
    bool valid(const VectorXd& residuals);

    /**
     * Verify if we should stop the optimization.
     *
     */
    bool stop();

    /*
     * Get the name of the strategy.
     */
    inline std::string name() const
    { return "RProp"; }

private:
    Options opt_;

    VectorXd delta_;
    VectorXd grad_;

    double chi2_;
    bool   stop_;
};

}} /* rtk::nlls */


//
// Copyright (c) 2018, IRIS Laboratory, IEETA, University of Aveiro - Portugal
//
// @date    2018-10-30
// @authors Eurico Pedrosa <efp@ua.pt>
//

#pragma once

#include <string>

#include <boost/shared_ptr.hpp>

#include <rtk/types.h>

namespace rtk  {
namespace nlls {

class Strategy {
public:

    struct Summary {

        /// The value of the trust radius.
        double trust_radius;

        /// Maximum absolute value of g = J'*r.
        double max_abs_g;

        /// Maximum absolute value of h from J'J*h = J'*r.
        double max_abs_h;

        /// Current cost of the problem.
        double cost;

        /// How long it took to perform the iteration step.
        double execution_time;
    };

    /**
     * Reset the optimization strategy to its initial state.
     */
    virtual void reset() = 0;

    /**
     * Performe an optimization step.
     *
     * @param[in]  residuals The problem residuals.
     * @param[in]  J         The Jacobian matrix.
     * @param[out] summary
     *
     * @returns The optimization step.
     */
    virtual VectorXd step(const VectorXd& residuals, const MatrixXd& J, Summary& summary) = 0;

    /**
     * Check the validity of the optimization step.
     *
     * @param[in] residuals The residuals after the optimization step.
     *
     * @returns True if the last step is valid, false otherwise.
     */
    virtual bool valid(const VectorXd& residuals) = 0;

    /**
     * Verify if we should stop the optimization.
     */
    virtual bool stop() = 0;

    /*
     * Get the name of the strategy.
     */
    virtual std::string name() const = 0;

public:
    typedef boost::shared_ptr<Strategy> Ptr;
};

}} /* rtk:nlls */


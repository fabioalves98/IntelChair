//
// Copyright (c) 2018, IRIS Laboratory, IEETA, University of Aveiro - Portugal
//
// @date    2018-10-30
// @authors Eurico Pedrosa <efp@ua.pt>
//

#pragma once

#include <rtk/types.h>

namespace rtk  {
namespace nlls {

class Problem {
public:

    /**
     *  Compute residuals and Jacobian.
     *  The Jacobian is only calculated if J is not null.
     *
     *  @param[out] residuals Computed residuals.
     *  @param[out] J         Jacobian matrix.
     */
    virtual void eval(VectorXd& residuals, MatrixXd* J) = 0;

    /**
     * Update the internal state.
     *
     * @param[in] h Optimization step.
     */
    virtual void update(const VectorXd& h) = 0;
};

}} /* rtk::nlls */


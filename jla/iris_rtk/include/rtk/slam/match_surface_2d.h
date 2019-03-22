//
// Copyright (c) 2018, IRIS Laboratory, IEETA, University of Aveiro - Portugal
//
// @date    2018-10-30
// @authors Eurico Pedrosa <efp@ua.pt>
//

#pragma once

#include <rtk/geom/lie.h>
#include <rtk/nlls/problem.h>

#include <rtk/sdm/dynamic_distance_map.h>

#include <vector>

namespace rtk  {
namespace slam {

class MatchSurface2D : public nlls::Problem {
public:

    MatchSurface2D(const sdm::DynamicDistanceMap*  surface,
                   const PointCloudXYZ::Ptr& scan,
                   const geom::SE2d& estimate);

    /**
     *
     */
    inline geom::SE2d getState() const
    { return state_; }

    /**
     *  Compute residuals and Jacobian.
     *  The Jacobian is only calculated if J is not null.
     *
     *  @param[out] residuals Computed residuals.
     *  @param[out] J         Jacobian matrix.
     */
    void eval(VectorXd& residuals, MatrixXd* J);

    /**
     * Update the internal state.
     *
     * @param[in] h Optimization step.
     */
    void update(const VectorXd& h);

private:
    const sdm::DynamicDistanceMap* surface_;
    PointCloudXYZ::Ptr      scan_;

    geom::SE2d state_;
};

}} /* rtk::slam */


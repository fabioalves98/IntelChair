//
// Copyright (c) 2018, IRIS Laboratory, IEETA, University of Aveiro - Portugal
//
// @date    2018-10-30
// @authors Eurico Pedrosa <efp@ua.pt>
//

#include <rtk/slam/match_surface_2d.h>

namespace rtk {
namespace slam {

MatchSurface2D::MatchSurface2D(const sdm::DynamicDistanceMap*  surface,
                   const PointCloudXYZ::Ptr& scan,
                   const geom::SE2d& estimate)
    : surface_(surface),
      scan_(scan),
      state_(estimate)
{ }

void MatchSurface2D::eval(VectorXd& residuals, MatrixXd* J)
{
    // 1. Transform the point cloud to the model coordinates.

    // Although we are working on a 2d plane the sensor is on a 3d plane.
    // Thus, to use the data points from the sensor we have project them into the 2d
    // plane of the moving frame and only then to the fixed 2d plane.
    Affine3d moving_tf = Translation3d(scan_->sensor_origin_) * scan_->sensor_orientation_;

    Vector3d trans; trans << state_.translation().x(),
                             state_.translation().y(),
                             0.0;
    Affine3d fixed_tf = Translation3d(trans) * AngleAxisd(state_.so2().log(), Vector3d::UnitZ());

    const size_t num_points = scan_->points.size();
    //== transform point cloud
    Affine3d tf = fixed_tf * moving_tf;

    // 2. Resize data holders
    Vector3d  hit;
    Vector3d  grad; // will only be used when J != nullptr

    residuals.resize(num_points);
    if (J != 0)
        J->resize(num_points, 3);

    // 3. Compute residuals and Jacobian (if required).
    for (size_t i = 0; i < num_points; ++i){
        hit = tf * scan_->points[i];
        hit[2] = 0.0;

        residuals[i] = surface_->distance(hit, &grad);

        if (J != 0)
            // the Jacobian of the euclidean distance (Je) is
            // equal to:
            //
            //     Je = | dx, dy |
            //
            // And the Jacobian of the special euclidean group (Js)
            // that transforms a point is given by :
            //
            //     Js = | 1, 0, -y |
            //          | 0, 1,  x |
            //
            // Then, the final Jacobian is equal to J = Je * Js
            J->row(i) << grad[0], grad[1], grad[1]*hit[0] - grad[0]*hit[1];
    }// end for
}

void MatchSurface2D::update(const VectorXd& h)
{
    // The state update in the manifold.
    state_ = geom::SE2d::exp(h) * state_;
}

}} /* rtk::slam */

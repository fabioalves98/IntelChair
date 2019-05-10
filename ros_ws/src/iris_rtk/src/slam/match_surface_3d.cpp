//
// Copyright (c) 2018, IRIS Laboratory, IEETA, University of Aveiro - Portugal
//
// @date    2018-10-30
// @authors Eurico Pedrosa <efp@ua.pt>
//

#include <rtk/slam/match_surface_3d.h>

namespace rtk {
namespace slam {

MatchSurface3D::MatchSurface3D(const sdm::DynamicDistanceMap*  surface,
                   const PointCloudXYZ::Ptr& scan,
                   const geom::SE3d& estimate)
    : surface_(surface),
      scan_(scan),
      state_(estimate)
{ }

void MatchSurface3D::eval(VectorXd& residuals, MatrixXd* J)
{
    // 1. Transform the point cloud to the model coordinates.
    Affine3d moving_tf = Translation3d(scan_->sensor_origin_) * scan_->sensor_orientation_;
    Affine3d fixed_tf(state_.matrix());

    PointCloudXYZ::Ptr cloud(new PointCloudXYZ);

    const size_t num_points = scan_->points.size();
    //== transform point cloud
    Affine3d tf = fixed_tf * moving_tf;
    cloud->points.reserve(num_points);
    for (size_t i = 0; i < num_points; ++i)
        cloud->points.push_back(tf * scan_->points[i]);
    //==

    // 2. Resize data holders
    Vector3d  hit;
    Vector3d  grad; // will only be used when J != nullptr

    residuals.resize(num_points);
    if (J != 0)
        J->resize(num_points, 6);

    // 3. Compute residuals and Jacobian (if required).
    for (size_t i = 0; i < num_points; ++i){
        hit = cloud->points[i];

        residuals[i] = surface_->distance(hit, &grad);

        if (J != 0){
            // the Jacobian of the euclidean distance (Je) is
            // equal to:
            //
            //     Je = | dx, dy, dz |
            //
            // And the Jacobian of the special euclidean group (Js)
            // that transforms a point is given by :
            //
            //     Js = | 1, 0, 0, 0, z,-y |
            //          | 0, 1, 0,-z, 0, x |
            //          | 0, 0, 1, y,-x, 0 |
            //
            // Then, the final Jacobian is equal to J = Je * Js

            /* Matrix<double, 3, 6> Js; */
            /* Js << 1, 0, 0,       0,  hit[2], -hit[1], */
            /*       0, 1, 0, -hit[2],       0,  hit[0], */
            /*       0, 0, 1,  hit[1], -hit[0],       0; */
            /* J->row(i) = grad.transpose() * Js; */

            J->row(i) << grad[0], grad[1], grad[2],
                         grad[2]*hit[1] - grad[1]*hit[2],
                         grad[0]*hit[2] - grad[2]*hit[0],
                         grad[1]*hit[0] - grad[0]*hit[1];
        }
    }// end for
}

void MatchSurface3D::update(const VectorXd& h)
{
    // The state update in the manifold.
    state_ = geom::SE3d::exp(h) * state_;
}

}} /* rtk::slam */

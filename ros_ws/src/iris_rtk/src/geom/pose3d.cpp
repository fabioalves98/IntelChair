//
// Copyright (c) 2018, IRIS Laboratory, IEETA, University of Aveiro - Portugal
//
// @date    2018-10-30
// @authors Eurico Pedrosa <efp@ua.pt>
//

#include <rtk/geom/pose3d.h>

namespace rtk  {
namespace geom {

Pose3D::Pose3D()
    : state_()
{}

Pose3D::Pose3D(const double& x, const double& y, const double& z,
               const double& roll, const double& pitch, const double& yaw)
{
    Quaterniond tmp = AngleAxisd(roll, Vector3d::UnitX()) *
                      AngleAxisd(pitch, Vector3d::UnitY()) *
                      AngleAxisd(yaw, Vector3d::UnitZ());

    state_.translation() = Vector3d(x,y,z);
    state_.setQuaternion(tmp);
}

Pose3D::Pose3D(const Vector3d& xyz, const double& yaw)
{
    Quaterniond tmp = AngleAxisd(0, Vector3d::UnitX()) *
                      AngleAxisd(0, Vector3d::UnitY()) *
                      AngleAxisd(yaw, Vector3d::UnitZ());

    state_.translation() = xyz;
    state_.setQuaternion(tmp);
}

Pose3D::Pose3D(const Vector3d& xyz, const Vector3d& rpy)
{
    Quaterniond tmp = AngleAxisd(rpy(0), Vector3d::UnitX()) *
                      AngleAxisd(rpy(1), Vector3d::UnitY()) *
                      AngleAxisd(rpy(2), Vector3d::UnitZ());

    state_.translation() = xyz;
    state_.setQuaternion(tmp);
}

Pose3D::Pose3D(const Matrix4d& transformation)
    : state_(transformation)
{}

Pose3D::Pose3D(const Affine3d& transformation)
{
    state_.translation() = transformation.translation();
    state_.setRotationMatrix(transformation.linear());
}

Pose3D::Pose3D(const Pose3D& other)
{
    state_ = other.state_;
}

Pose3D::Pose3D(const SE3d& se3)
    : state_(se3)
{}

Pose3D::~Pose3D()
{}

Pose3D Pose3D::operator+(const Pose3D& other)
{
    return Pose3D(state_ * other.state_);
}

Pose3D Pose3D::operator-(const Pose3D& other)
{
    return Pose3D(state_.inverse() * other.state_);
}

Pose3D& Pose3D::operator+=(const Pose3D& other)
{
    state_ *= other.state_;
    return *this;
}

Pose3D& Pose3D::operator-=(const Pose3D& other)
{
    state_ = state_.inverse() * other.state_;
    return *this;
}

Vector3d Pose3D::operator*(const Vector3d& point)
{
    return state_*point;
}

double Pose3D::x() const
{
    return state_.translation().x();
}

double Pose3D::y() const
{
    return state_.translation().y();
}

double Pose3D::z() const
{
    return state_.translation().z();
}

Vector3d Pose3D::xyz() const
{
    return state_.translation();
}

double Pose3D::roll() const
{
    return rpy().x();
}

double Pose3D::pitch() const
{
    return rpy().y();
}

double Pose3D::yaw() const
{
    return rpy().z();
}

Vector3d Pose3D::rpy() const
{
    return state_.rotationMatrix().eulerAngles(0,1,2);
}

}} /* rtk::geom */

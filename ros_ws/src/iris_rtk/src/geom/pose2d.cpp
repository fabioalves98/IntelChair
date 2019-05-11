//
// Copyright (c) 2018, IRIS Laboratory, IEETA, University of Aveiro - Portugal
//
// @date    2018-10-30
// @authors Eurico Pedrosa <efp@ua.pt>
//

#include <rtk/geom/pose2d.h>

namespace rtk  {
namespace geom {

Pose2D::Pose2D()
    : state_()
{}

Pose2D::Pose2D(const double& x, const double& y, const double& rotation)
    : state_(rotation, Vector2d(x,y))
{}

Pose2D::Pose2D(const Vector2d& xy, const double& rotation)
    : state_(rotation, xy)
{}

Pose2D::Pose2D(const Vector3d& xyr)
    : state_(xyr[2], xyr.tail<2>())
{}

Pose2D::Pose2D(const Affine2d& transformation)
{
    state_.translation() = transformation.translation();
    state_.setRotationMatrix(transformation.linear());
}

Pose2D::Pose2D(const Pose2D& other)
{
    state_ = other.state_;
}

Pose2D::Pose2D(const SE2d& se2)
    : state_(se2)
{}

Pose2D::~Pose2D()
{}

Pose2D& Pose2D::operator=(const Pose2D& other)
{
    state_ = other.state_;
    return *this;
}

Pose2D Pose2D::operator+(const Pose2D& other)
{
    return Pose2D(state_ * other.state_);
}

Pose2D Pose2D::operator-(const Pose2D& other)
{
    return Pose2D(state_.inverse() * other.state_);
}

Pose2D& Pose2D::operator+=(const Pose2D& other)
{
    state_ *= other.state_;
    return *this;
}

Pose2D& Pose2D::operator-=(const Pose2D& other)
{
    state_ = state_.inverse() * other.state_;
    return *this;
}

Vector2d Pose2D::operator*(const Vector2d& point)
{
    return state_*point;
}

double Pose2D::x() const
{
    return state_.translation().x();
}

double Pose2D::y() const
{
    return state_.translation().y();
}

Vector2d Pose2D::xy() const
{
    return state_.translation();
}

double Pose2D::rotation() const
{
    return state_.so2().log();
}

const Vector3d Pose2D::xyr() const
{
    Vector3d tmp;
    tmp.x() = state_.translation().x();
    tmp.y() = state_.translation().y();
    tmp.z() = state_.so2().log();
    return tmp;
}

}} /* rtk::geom */

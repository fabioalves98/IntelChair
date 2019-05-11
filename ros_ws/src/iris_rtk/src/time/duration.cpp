//
// Copyright (c) 2018, IRIS Laboratory, IEETA, University of Aveiro - Portugal
//
// @date    2018-10-30
// @authors Eurico Pedrosa <efp@ua.pt>
//

#include <thread>

#include <rtk/time/duration.h>

namespace rtk  {
namespace time {

Duration::Duration()
    : duration_(std::chrono::nanoseconds::zero())
{}

Duration::Duration(const std::chrono::nanoseconds& nano)
    : duration_(nano)
{}

Duration::Duration(double seconds)
    : duration_((int64_t)(seconds*1.e9))
{}

bool Duration::isZero() const
{
    return duration_ == std::chrono::nanoseconds::zero();
}

bool Duration::sleep() const
{
    std::this_thread::sleep_for(duration_);
    return true;
}

double Duration::toSec() const
{
    double sec  = duration_.count()/1.e9;
    return sec;
}

int64_t Duration::toNSec() const
{
    return duration_.count();
}

Duration& Duration::operator=(const Duration& rhs)
{
    duration_ = rhs.duration_;
    return *this;
}

bool Duration::operator==(const Duration& rhs) const
{
    return (duration_ == rhs.duration_);
}

bool Duration::operator!=(const Duration& rhs) const
{
    return (duration_ != rhs.duration_);
}

bool Duration::operator<=(const Duration& rhs) const
{
    return (duration_ <= rhs.duration_);
}

bool Duration::operator>=(const Duration& rhs) const
{
    return (duration_ >= rhs.duration_);
}

bool Duration::operator<(const Duration& rhs) const
{
    return (duration_ < rhs.duration_);
}

bool Duration::operator>(const Duration& rhs) const
{
    return (duration_ > rhs.duration_);
}

Duration Duration::operator+(const Duration& rhs) const
{
    return Duration(duration_ + rhs.duration_);
}

Duration Duration::operator-(const Duration& rhs) const
{
    return Duration(duration_ - rhs.duration_);
}

Duration Duration::operator-() const
{
    return Duration(-duration_);
}

Duration& Duration::operator+=(const Duration& rhs)
{
    duration_ += rhs.duration_;
    return *this;
}

Duration& Duration::operator-=(const Duration& rhs)
{
    duration_ -= rhs.duration_;
    return *this;
}

}} /* rtk::time  */

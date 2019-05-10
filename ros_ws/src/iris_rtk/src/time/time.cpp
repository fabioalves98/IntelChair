//
// Copyright (c) 2018, IRIS Laboratory, IEETA, University of Aveiro - Portugal
//
// @date    2018-10-30
// @authors Eurico Pedrosa <efp@ua.pt>
//

#include <rtk/time/time.h>

namespace rtk  {
namespace time {

Time::Time()
    : duration_()
{}

Time::Time(const Duration& duration)
    : duration_(duration)
{}

Time::Time(double seconds)
    : duration_(seconds)
{}

Time Time::now()
{
    return Time( Duration(std::chrono::system_clock::now().time_since_epoch()) );
}

bool Time::sleepUntil(const Time& end)
{
    Duration d = end - Time::now();
    if (d > Duration(0))
        return d.sleep();

    return true;
}

bool Time::isZero() const
{
    return duration_.isZero();
}

double Time::toSec() const
{
    return duration_.toSec();
}

int64_t Time::toNSec() const
{
    return duration_.toNSec();
}

bool Time::operator==(const Time& rhs) const
{
    return duration_ == rhs.duration_;
}

bool Time::operator!=(const Time& rhs) const
{
    return duration_ != rhs.duration_;
}

bool Time::operator<=(const Time& rhs) const
{
    return duration_ <= rhs.duration_;
}

bool Time::operator>=(const Time& rhs) const
{
    return duration_ >= rhs.duration_;
}

bool Time::operator<(const Time& rhs) const
{
    return duration_ < rhs.duration_;
}

bool Time::operator>(const Time& rhs) const
{
    return duration_ > rhs.duration_;
}

Duration Time::operator-(const Time& rhs) const
{
    return duration_ - rhs.duration_;
}

Time Time::operator-(const Duration& rhs) const
{
    Time t = *this;
    t.duration_ += (-rhs);
    return t;
}

Time Time::operator+(const Duration& rhs) const
{
    Time t = *this;
    t.duration_ += rhs;
    return t;
}

}} /* tk::time  */

//
// Copyright (c) 2018, IRIS Laboratory, IEETA, University of Aveiro - Portugal
//
// @date    2018-10-30
// @authors Eurico Pedrosa <efp@ua.pt>
//

#pragma once

#include <rtk/time/duration.h>

namespace rtk  {
namespace time {

class Time {
public:
    Time();
    Time(const Duration& duration);

    explicit Time(double seconds);

    static Time now();

    static bool sleepUntil(const Time& end);

    /**
     * Check for zero Time.
     */
    bool isZero() const;

    /**
     * Convert the amount of time specified by this Time to seconds.
     */
    double toSec() const;

    /**
     * Convert the amount of time specified by this Time to nanoseconds.
     */
    int64_t toNSec() const;

    /* Time& operator=(const Time& rhs); */

    bool operator==(const Time& rhs) const;
    bool operator!=(const Time& rhs) const;

    bool operator<=(const Time& rhs) const;
    bool operator>=(const Time& rhs) const;

    bool operator<(const Time& rhs) const;
    bool operator>(const Time& rhs) const;

    Duration operator-(const Time& rhs) const;
    Time operator-(const Duration& rhs) const;
    Time operator+(const Duration& rhs) const;

private:
    Duration duration_;
};


}} /* rtk::time  */


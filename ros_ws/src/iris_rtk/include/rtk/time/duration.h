//
// Copyright (c) 2018, IRIS Laboratory, IEETA, University of Aveiro - Portugal
//
// @date    2018-10-30
// @authors Eurico Pedrosa <efp@ua.pt>
//

#pragma once

#include <chrono>

#include <rtk/types.h>

namespace rtk  {
namespace time {

/**
 * Duration representation for use with Time class.
 *
 */
class Duration {
public:
    /**
     * Create a *zero* duration.
     */
    Duration();

    /**
     * Construct a Duration.
     */
    Duration(const std::chrono::nanoseconds& nano);

    /**
     * Construct a Duration.
     *
     * @param seconds Amount of seconds.
     */
    explicit Duration(double seconds);

    /**
     * Check for zero duration.
     */
    bool isZero() const;


    /**
     * Sleep for the amount of time specified by this Duration.
     *
     */
    bool sleep() const;

    /**
     * Convert the amount of time specified by this Duration to seconds.
     */
    double toSec() const;

    /**
     * Convert the amount of time specified by this Duration to nanoseconds.
     */
    int64_t toNSec() const;

    Duration& operator=(const Duration& rhs);

    bool operator==(const Duration& rhs) const;
    bool operator!=(const Duration& rhs) const;

    bool operator<=(const Duration& rhs) const;
    bool operator>=(const Duration& rhs) const;

    bool operator<(const Duration& rhs) const;
    bool operator>(const Duration& rhs) const;

    Duration operator+(const Duration& rhs) const;
    Duration operator-(const Duration& rhs) const;
    Duration operator*(double scale) const;
    Duration operator-() const;

    Duration& operator+=(const Duration& rhs);
    Duration& operator-=(const Duration& rhs);

private:
    std::chrono::nanoseconds duration_;
};

}} /* rtk::time  */


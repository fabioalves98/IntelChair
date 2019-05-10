//
// Copyright (c) 2018, IRIS Laboratory, IEETA, University of Aveiro - Portugal
//
// @date    2018-10-30
// @authors Eurico Pedrosa <efp@ua.pt>
//

#pragma once

#include <queue>

#include <rtk/time/timer.h>

namespace rtk   {
namespace time {

class Frequency {
public:

    Frequency(size_t window_size = 30);

    /**
     * Notify the class that an event occured.
     */
    void event();
    void event(const double& timestamp);

    /**
     * Get the estimated frequency.
     */
    double getFrequency() const;

    /**
     * Reset frequency computation.
     */
    void reset();

private:
    const size_t         window_;
    std::queue<Duration> event_queue_;
    Timer                timer_;

};

}} /* rtk::time */


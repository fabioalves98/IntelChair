//
// Copyright (c) 2018, IRIS Laboratory, IEETA, University of Aveiro - Portugal
//
// @date    2018-10-30
// @authors Eurico Pedrosa <efp@ua.pt>
//

#include <rtk/time/rate.h>

namespace rtk  {
namespace time {

Rate::Rate(double frequency)
    : start_(Time::now()),
      cycle_(1.0 / frequency),
      actual_cycle_(0)
{}

void Rate::reset()
{
    start_ = Time::now();
}

bool Rate::sleep()
{
    Time end = start_ + cycle_;
    Time now = Time::now();

    // time travel ?
    if (now < start_)
        end = now + cycle_;

    Duration sleep = end - now;
    // save the ammount time spent in the cycle
    actual_cycle_ = now - start_;
    // reset the time
    start_ = end;

    // if we took to much time, skip sleeping.
    if (sleep <= Duration(0.0)){

        // reset time if the loop has taken more than a full cycle,
        // or jumped in time.
        if (now > end + cycle_)
            start_ = now;

        return true;
    }


    return sleep.sleep();
}

Duration Rate::actualCycle() const
{
    return actual_cycle_;
}

}} /* rtk::time */

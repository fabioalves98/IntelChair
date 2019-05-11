//
// Copyright (c) 2018, IRIS Laboratory, IEETA, University of Aveiro - Portugal
//
// @date    2018-10-30
// @authors Eurico Pedrosa <efp@ua.pt>
//

#include <rtk/time/frequency.h>

namespace rtk  {
namespace time {

Frequency::Frequency(size_t window_size)
    : window_(window_size)
{
    timer_.reset();
}

void Frequency::event()
{
    event_queue_.push(timer_.elapsed());
    if (event_queue_.size() > window_)
        event_queue_.pop();
}

void Frequency::event(const double& timestamp)
{
    event_queue_.push(Duration(timestamp));
    if (event_queue_.size() > window_)
        event_queue_.pop();
}

double Frequency::getFrequency() const
{
    if (event_queue_.size() < 2)
        return 0.0;

    double diff = (event_queue_.back() - event_queue_.front()).toSec();

    return (event_queue_.size()-1) / diff;
}

void Frequency::reset()
{
    timer_.reset();
    event_queue_ = std::queue<Duration>();
}

}} /* tk::time */

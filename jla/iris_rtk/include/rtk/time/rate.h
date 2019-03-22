//
// Copyright (c) 2018, IRIS Laboratory, IEETA, University of Aveiro - Portugal
//
// @date    2018-10-30
// @authors Eurico Pedrosa <efp@ua.pt>
//

#pragma once

#include <rtk/time/time.h>

namespace rtk  {
namespace time {

class Rate {
public:

    Rate(double frequency);

    void reset();
    bool sleep();

    Duration actualCycle() const;

private:
    Time start_;
    Duration cycle_;
    Duration actual_cycle_;
};


}} /* rtk::time */


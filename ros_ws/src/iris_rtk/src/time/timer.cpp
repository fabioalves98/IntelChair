//
// Copyright (c) 2018, IRIS Laboratory, IEETA, University of Aveiro - Portugal
//
// @date    2018-10-30
// @authors Eurico Pedrosa <efp@ua.pt>
//

#include <rtk/time/timer.h>

namespace rtk  {
namespace time {

using namespace std;

Timer::Timer(bool immediately)
{
    if (immediately)
        reset();
}

void Timer::reset()
{
    start_ = clock::now();
}

Duration Timer::elapsed() const
{
    return Duration(clock::now() - start_);
}

}} /* tk::time  */

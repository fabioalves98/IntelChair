//
// Copyright (c) 2018, IRIS Laboratory, IEETA, University of Aveiro - Portugal
//
// @date    2018-10-30
// @authors Eurico Pedrosa <efp@ua.pt>
//

#ifndef RTK_TIME_TIMER_H_
#define RTK_TIME_TIMER_H_

#include <rtk/time/time.h>

namespace rtk  {
namespace time {

class Timer {
public:
    Timer(bool immediately = false);

    inline void start()
    { reset(); }

    void reset();
    Duration elapsed() const;

private:
    typedef std::chrono::high_resolution_clock clock;
    clock::time_point start_;
};

}} /* rtk::time  */

#endif /* end of include guard: RTK_TIME_TIMER_H_ */

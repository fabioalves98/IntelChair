//
// Copyright (c) 2018, IRIS Laboratory, IEETA, University of Aveiro - Portugal
//
// @date    2018-10-01
// @authors Eurico Pedrosa <efp@ua.pt>
//

#pragma once

#include "rtk/_vendor/concurrentqueue.h"

namespace rtk {

template <typename T>
using ConcurrentQueue = moodycamel::ConcurrentQueue<T>;

} // namespace rtk


//
// Copyright (c) 2018, IRIS Laboratory, IEETA, University of Aveiro - Portugal
//
// @date    2018-10-30
// @authors Eurico Pedrosa <efp@ua.pt>
//

#pragma once

#include <string>

namespace rtk {

// Output a formated string to stdout.
void print(const char* format, ...) __attribute__((__format__ (__printf__, 1, 2)));

// Create a string from a formated text.
std::string format(const char* format, ...) __attribute__((__format__ (__printf__, 1, 2)));

} // namespace rtk


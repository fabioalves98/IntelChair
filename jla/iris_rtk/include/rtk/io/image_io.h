//
// Copyright (c) 2018, IRIS Laboratory, IEETA, University of Aveiro - Portugal
//
// @date    2018-10-30
// @authors Eurico Pedrosa <efp@ua.pt>
//

#pragma once

#include <string>

#include "rtk/structs/image.h"

namespace rtk {
namespace io  {

bool image_write_png(const Image& image, const std::string& filename);

}} // namespace rtk::io


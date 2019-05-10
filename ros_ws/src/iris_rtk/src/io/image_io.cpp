//
// Copyright (c) 2018, IRIS Laboratory, IEETA, University of Aveiro - Portugal
//
// @date    2018-10-30
// @authors Eurico Pedrosa <efp@ua.pt>
//

#include <fstream>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "rtk/_vendor/stb/stb_image_write.h"

#include "rtk/io/image_io.h"

bool rtk::io::image_write_png(const Image& image, const std::string& filename)
{
    uint32_t stride = image.width * sizeof(uint8_t) * image.channels;
    int ret = stbi_write_png(filename.c_str(),
                             image.width, image.height, image.channels,
                             image.data.get(), stride);

    return ret == 0;
}


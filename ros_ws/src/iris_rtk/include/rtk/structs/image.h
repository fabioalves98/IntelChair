//
// Copyright (c) 2018, IRIS Laboratory, IEETA, University of Aveiro - Portugal
//
// @date    2018-08-07
// @authors Eurico Pedrosa <efp@ua.pt>
//

#pragma once

#include <memory>

#include "rtk/types.h"

namespace rtk {

/// Data structure for images. It it is modele to work with
/// stb's image manipulation functions (https//github.com/nothings/stb/).
/// Most of the comments are borrowed from stb's source code.
struct Image {

    /// An image is a rectangle of pixels stored from left-to-right, top-to-bottom.
    /// The data pointer points to the first top-left-most pixel.
    /// The data can only be retained by a single object.
    std::unique_ptr<uint8_t[]> data;

    /// Each pixel contains N channels of data stored interleaved
    /// with 8 bits per channel, with the following order:
    ///     1 = Grayscale, 2 = Grayscale + Alpha, 3 = RGB, 4 = RGBA
    uint32_t channels;

    /// An image has `width` pixels wide.
    uint32_t width;
    /// An image has `height` pixels tall.
    uint32_t height;

    /// Check the image is valid, or not.
    inline bool ok() const
    { return data != nullptr; }

    /// Allocate data for an image of width `w`, height `h` and channels `c`.
    inline void alloc(uint32_t w, uint32_t h, uint32_t c)
    {
        width = w; height = h; channels = c;
        data.reset(new uint8_t[width*height*channels]);
    }

    /// Fill all pixels (and channels) with the same value.
    inline void fill(uint8_t value)
    { std::memset(data.get(), value, width * height * channels); }

    /// Access to a pixel by u,v and channel.
    /// @Warning There is no validation of the input parameters.
    inline uint8_t& operator()(uint32_t u, uint32_t v, uint32_t c = 0)
    { return data[(u * channels + v * width*channels) + c]; }

};

} // namespace iris


//
// Copyright (c) 2018, IRIS Laboratory, IEETA, University of Aveiro - Portugal
//
// @date    2018-10-30
// @authors Eurico Pedrosa <efp@ua.pt>
//

#include "../_vendor/ldc/quicklz/quicklz.h"
#include "rtk/ldc/quicklz_buffer_compressor.h"

namespace rtk {
namespace ldc {

size_t QuickLZBufferCompressor::compress(const char* src, size_t src_size, char** dst, char* buffer)
{
    size_t dst_size = src_size + 400;
    bool free_buffer = true;

    if (buffer == 0)
        buffer = new char[dst_size];
    else
        free_buffer = false;

    qlz_state_compress *state_compress = (qlz_state_compress *)malloc(sizeof(qlz_state_compress));

    dst_size = qlz_compress(src, buffer, src_size, state_compress);

    free(state_compress);

    if (dst_size > 0){
        *dst = new char[dst_size];
        memcpy(*dst, buffer, dst_size);
    } else {
        dst_size = 0;
    }

    if (free_buffer)
        delete [] buffer;

    return dst_size;
}

size_t QuickLZBufferCompressor::decompress(const char* src, size_t src_size, char** dst, size_t dst_size)
{
    *dst = new char[dst_size];

    qlz_state_decompress *state_decompress = (qlz_state_decompress *)malloc(sizeof(qlz_state_decompress));

    int ret = qlz_decompress(src, *dst, state_decompress);

    free(state_decompress);

    if (ret <= 0){
        delete [] *dst;
        *dst = 0;
        return 0;
    }

    return dst_size;
}

size_t QuickLZBufferCompressor::compressBound(size_t size)
{
    return size + 400;
}

BufferCompressor* QuickLZBufferCompressor::clone() const
{
    return new QuickLZBufferCompressor();
}

}} // rtk::ldc

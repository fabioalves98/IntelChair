//
// Copyright (c) 2018, IRIS Laboratory, IEETA, University of Aveiro - Portugal
//
// @date    2018-10-30
// @authors Eurico Pedrosa <efp@ua.pt>
//

#include "../_vendor/ldc/pithy/pithy.h"
#include "rtk/ldc/pithy_buffer_compressor.h"

namespace rtk {
namespace ldc {

size_t PithyBufferCompressor::compress(const char* src, size_t src_size, char** dst, char* buffer)
{
    const int max_destiny_size = pithy_MaxCompressedLength(src_size);
    bool free_buffer = true;

    if (buffer == 0)
        buffer = new char[max_destiny_size];
    else
        free_buffer = false;

    int compressed_size = pithy_Compress(src, src_size, buffer, max_destiny_size, 0);

    if (compressed_size > 0){
        *dst = new char[compressed_size];
        memcpy(*dst, buffer, compressed_size);
    }

    if (free_buffer)
        delete [] buffer;

    return (size_t)compressed_size;
}

size_t PithyBufferCompressor::decompress(const char* src, size_t src_size, char** dst, size_t dst_size)
{
    *dst = new char[dst_size];

    int compressed_size = pithy_Decompress(src, src_size, *dst, dst_size);

    if (compressed_size < 0){
        delete [] *dst;
        *dst = 0;
        return 0;
    }

    return dst_size;
}

size_t PithyBufferCompressor::compressBound(size_t size)
{
    return pithy_MaxCompressedLength(size);
}

BufferCompressor* PithyBufferCompressor::clone() const
{
    return new PithyBufferCompressor();
}

}} // rtk::ldc

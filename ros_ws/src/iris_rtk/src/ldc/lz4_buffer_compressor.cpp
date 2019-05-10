//
// Copyright (c) 2018, IRIS Laboratory, IEETA, University of Aveiro - Portugal
//
// @date    2018-10-30
// @authors Eurico Pedrosa <efp@ua.pt>
//

#include "../_vendor/ldc/lz4/lz4.h"
#include "rtk/ldc/lz4_buffer_compressor.h"

namespace rtk {
namespace ldc {

size_t LZ4BufferCompressor::compress(const char* src, size_t src_size, char** dst, char* buffer)
{
    const int max_destiny_size = LZ4_compressBound(src_size);
    bool free_buffer = true;

    if (buffer == 0)
        buffer = new char[max_destiny_size];
    else
        free_buffer = false;

    int compressed_size = LZ4_compress_fast(src, buffer, src_size, max_destiny_size, 1);

    if (compressed_size > 0){
        *dst = new char[compressed_size];
        memcpy(*dst, buffer, compressed_size);
    }

    if (free_buffer)
        delete [] buffer;

    return (size_t)compressed_size;
}

size_t LZ4BufferCompressor::decompress(const char* src, size_t src_size, char** dst, size_t dst_size)
{
    *dst = new char[dst_size];

    int compressed_size = LZ4_decompress_fast(src, *dst, dst_size);

    if (compressed_size < 0){
        delete [] *dst;
        *dst = 0;
        return 0;
    }

    return dst_size;
}

size_t LZ4BufferCompressor::compressBound(size_t size)
{
    return LZ4_compressBound(size);
}

BufferCompressor* LZ4BufferCompressor::clone() const
{
    return new LZ4BufferCompressor();
}

}} // rtk::ldc

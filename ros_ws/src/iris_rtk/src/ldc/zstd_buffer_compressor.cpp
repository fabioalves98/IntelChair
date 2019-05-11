//
// Copyright (c) 2018, IRIS Laboratory, IEETA, University of Aveiro - Portugal
//
// @date    2018-10-30
// @authors Eurico Pedrosa <efp@ua.pt>
//

#include "../_vendor/ldc/zstd/zstd.h"
#include "rtk/ldc/zstd_buffer_compressor.h"

namespace rtk {
namespace ldc {

ZSTDBufferCompressor::ZSTDBufferCompressor()
{
    zstd_cc_ = ZSTD_createCCtx();
    zstd_dc_ = ZSTD_createDCtx();
}

ZSTDBufferCompressor::~ZSTDBufferCompressor()
{
    ZSTD_freeCCtx(zstd_cc_);
    ZSTD_freeDCtx(zstd_dc_);
}

size_t ZSTDBufferCompressor::compress(const char* src, size_t src_size, char** dst, char* buffer)
{
    const size_t max_destiny_size = ZSTD_compressBound(src_size);
    bool free_buffer = true;

    if (buffer == 0)
        buffer = new char[max_destiny_size];
    else
        free_buffer = false;

    int compressed_size = ZSTD_compressCCtx(zstd_cc_, buffer, max_destiny_size, src, src_size, 1);

    if (compressed_size > 0){
        *dst = new char[compressed_size];
        memcpy(*dst, buffer, compressed_size);
    }

    if (free_buffer)
        delete [] buffer;

    return (size_t)compressed_size;
}

size_t ZSTDBufferCompressor::decompress(const char* src, size_t src_size, char** dst, size_t dst_size)
{
    *dst = new char[dst_size];

    int compressed_size = ZSTD_decompressDCtx(zstd_dc_, *dst, dst_size, src, src_size);

    if (compressed_size <= 0){
        delete [] *dst;
        *dst = 0;
        return 0;
    }

    return dst_size;
}

size_t ZSTDBufferCompressor::compressBound(size_t size)
{
    return ZSTD_compressBound(size);
}

BufferCompressor* ZSTDBufferCompressor::clone() const
{
    return new ZSTDBufferCompressor();
}

}} // rtk::ldc

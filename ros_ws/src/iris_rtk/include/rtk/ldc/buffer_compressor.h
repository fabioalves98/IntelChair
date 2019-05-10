//
// Copyright (c) 2018, IRIS Laboratory, IEETA, University of Aveiro - Portugal
//
// @date    2018-10-30
// @authors Eurico Pedrosa <efp@ua.pt>
//

#pragma once

#include <rtk/types.h>

namespace rtk {
namespace ldc {

/**
 * Abstract in-memory buffer compressor.
 */
class BufferCompressor {
public:

    virtual ~BufferCompressor(){}

    /**
     * Compress a memory buffer.
     *
     * @param[in]  src       Memory buffer to be compressed.
     * @param[in]  src_size  The size in bytes of the uncompressed memory buffer.
     * @param[out] dst       Memory buffer that will hold the compressed data.
     *                       The buffer must _not_ be allocated, it will be allocated
     *                       by the function.
     * @param[in]  buffer    Intermediary buffer used during compression. Make
     *                       sure that its size is >= compressBound(@p src_size).
     *                       If buffer is *null* an internal one will be used.
     *
     * @return Number of bytes written into @p dst or 0 if the compression fails.
     */
    virtual size_t compress(const char* src, size_t src_size, char** dst, char* buffer = 0) = 0;

    /**
     * Decompress a memory buffer.
     */
    virtual size_t decompress(const char* src, size_t src_size, char** dst, size_t dst_size) = 0;

    /**
     * Calculate the maximum compressed size in worst case scenario.
     *
     * @param[in] size
     *
     * @return Maximum compressed size.
     */
    virtual size_t compressBound(size_t size) = 0;

    /**
     * Create a clone of the compressor.
     */
    virtual BufferCompressor* clone() const = 0;
};

}} // rtk::ldc


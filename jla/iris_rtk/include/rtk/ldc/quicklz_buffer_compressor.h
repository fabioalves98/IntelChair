//
// Copyright (c) 2018, IRIS Laboratory, IEETA, University of Aveiro - Portugal
//
// @date    2018-10-30
// @authors Eurico Pedrosa <efp@ua.pt>
//

#pragma once

#include <rtk/ldc/buffer_compressor.h>

namespace rtk {
namespace ldc {

class QuickLZBufferCompressor : public BufferCompressor {
public:

    virtual ~QuickLZBufferCompressor(){}

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
    size_t compress(const char* src, size_t src_size, char** dst, char* buffer = 0);

    /**
     * Decompress a memory buffer.
     */
    size_t decompress(const char* src, size_t src_size, char** dst, size_t dst_size);

    /**
     * Calculate the maximum compressed size in worst case scenario.
     *
     * @param[in] size
     *
     * @return Maximum compressed size.
     */
    size_t compressBound(size_t size);

    BufferCompressor* clone() const;

};

}} // rtk::ldc


//
// Copyright (c) 2018, IRIS Laboratory, IEETA, University of Aveiro - Portugal
//
// @date    2018-10-30
// @authors Eurico Pedrosa <efp@ua.pt>
//

#pragma once

#include "rtk/types.h"
#include "rtk/ldc/buffer_compressor.h"

namespace rtk {
namespace sdm {

// Data container that holds a fixed number of elements.
// The allocated data can be compressed/decompressed on request.
// In practice you can store any type of element (i.e. data stucture),
// however, keep in mind that nay constructors and/or destructors
// will never be called, the container works only with plain c structures.
// Furthermore, all allocated data are initiallized with zero.
class Container {
public:

    // The data is stored in a continuous chunk of memory.
    uint8_t* data = nullptr;
    // And we keep track of allocated data and element size.
    uint32_t memory_size  = 0;
    uint32_t element_size = 0;
    // But because data can be compressed we need its actual size;
    uint32_t actual_memory_size = 0;

    Container() = default;
    Container(const Container& other);

    virtual ~Container();

    /**
     * Verify if the container is already allocated.
     */
    bool ok() const;

    // Allocate memory for the container.
    // The allocated memory is set to zero. The `calloc` function is used
    // for this purpose. [Hopefully] copy-on-write is used to zero out the memory.
    // Return true on success.
    bool alloc(uint32_t size, uint32_t size_of_element);

    /**
     *
     */
    size_t memory() const;

    /**
     *
     */
    size_t fullMemory() const;

    /**
     * Access data by index.
     *
     * This is an unsafe function because it does not verify
     * if the container is already allocated and the index in within
     * bounds.
     *
     * @returns A pointer to the data.
     */
    //T* get(uint32_t idx);

    inline uint8_t* get(uint32_t idx)
    { return (data + idx * element_size); }

    /**
     * Access data by index (constant version).
     *
     * This is an unsafe function because it does not verify
     * if the container is already allocated and the index in within
     * bounds.
     *
     * @returns A constant pointer to the data.
     */
    //const T* get(uint32_t idx) const;

    const uint8_t* get(uint32_t idx) const
    { return (data + idx * element_size); }

    bool compress(ldc::BufferCompressor* bc, char* buffer);
    bool decompress(ldc::BufferCompressor* bc);

    inline bool isCompressed() const
    { return actual_memory_size != memory_size; }

    void write(ldc::BufferCompressor* bc, std::ofstream& stream) const;
    void read(std::ifstream& stream);
};

}} // namespace rtk::sdm


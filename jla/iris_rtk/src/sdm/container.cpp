//
// Copyright (c) 2018, IRIS Laboratory, IEETA, University of Aveiro - Portugal
//
// @date    2018-10-30
// @authors Eurico Pedrosa <efp@ua.pt>
//

#include "rtk/print.h"
#include "rtk/sdm/container.h"

rtk::sdm::Container::Container(const Container& other)
{
    if (other.data == nullptr)
        return;

    memory_size  = other.memory_size;
    element_size = other.element_size;
    actual_memory_size = other.actual_memory_size;

    data = (uint8_t*) malloc(actual_memory_size);
    memcpy(data, other.data, actual_memory_size );
}

rtk::sdm::Container::~Container()
{
    free(data);
}

bool rtk::sdm::Container::ok() const
{
    return (data != nullptr);
}

bool rtk::sdm::Container::alloc(uint32_t size, uint32_t size_of_element)
{
    data = static_cast<uint8_t*>(calloc(size, size_of_element));
    if (data == nullptr)
        return false;

    memory_size  = size * size_of_element;
    element_size = size_of_element;
    actual_memory_size = memory_size;
}

size_t rtk::sdm::Container::memory() const
{
    return actual_memory_size;
}

size_t rtk::sdm::Container::fullMemory() const
{
    return memory_size;
}

bool rtk::sdm::Container::compress(ldc::BufferCompressor* bc, char* buffer)
{
    if (memory_size != actual_memory_size)
        return true; // already compressed

    char* new_data;
    auto compressed_size = bc->compress((const char*)data, memory_size, &new_data, buffer);

    if (compressed_size == 0)
        return false;

    free(data);
    data = (uint8_t*)new_data;
    actual_memory_size = compressed_size;

    return true;
}

bool rtk::sdm::Container::decompress(ldc::BufferCompressor* bc)
{
    if (memory_size == actual_memory_size)
        return true; // already decompressed

    char* new_data;
    auto expected_size = bc->decompress((const char*)data, actual_memory_size, &new_data, memory_size);

    if (expected_size != memory_size)
        return false;

    free(data);
    data = (uint8_t*)new_data;
    actual_memory_size = memory_size;

    return true;
}

void rtk::sdm::Container::write(ldc::BufferCompressor* bc, std::ofstream& stream) const
{
#if 0
    ldc::ZSTDBufferCompressor zstd;

    if (compressed_size_ > 0){
        // data is already compressed, must decompress and recompress with zstd

        size_t dst_size = mem_size_ * sizeof(T);
        char* rbuffer;
        char* cbuffer;

        bc->decompress(cdata_, compressed_size_, &rbuffer, dst_size);
        dst_size = zstd.compress(rbuffer, dst_size, &cbuffer);

        stream.write((char*)&dst_size, sizeof(dst_size));
        stream.write(cbuffer, dst_size);

        delete [] rbuffer;
        delete [] cbuffer;

    } else {
        // lets compress the data and then write
        char* out;
        int cs = zstd.compress((const char*)data_, mem_size_ * sizeof(T), &out);
        stream.write((char*)&cs, sizeof(cs));
        stream.write(out, cs);
    }
#endif
}

void rtk::sdm::Container::read(std::ifstream& stream)
{
#if 0
    free(data_);
    data_ = 0;

    stream.read((char*)&compressed_size_, sizeof(int));

    cdata_ = new char[compressed_size_];
    stream.read(cdata_, compressed_size_);
#endif
}


//
// Copyright (c) 2018, IRIS Laboratory, IEETA, University of Aveiro - Portugal
//
// @date    2018-10-30
// @authors Eurico Pedrosa <efp@ua.pt>
//

#pragma once

// stl includes
#include <utility>

// local includes
#include <rtk/types.h>
#include <rtk/structs/cow_ptr.h>

#include <rtk/ldc/buffer_compressor.h>

#include "rtk/sdm/container.h"

namespace rtk {
namespace sdm {

/**
 * Base class for any type of Map.
 *
 * A **Map** is a homogenous discrete 2D or 3D representation of the environment.
 * Its unit is a **cell** that can represent any type of information. Because
 * the dimensions of a map is hardly known in dynamic applications (e.g. SLAM),
 * a map is sub-divided into smaller maps, called **patches**, that are only
 * allocated when a cell within its bounds is used.
 *
 */
class Map {
public:

    // The larger integer coordinate that a patch can have is ruled by the universal
    // constant, i.e. index < UNIVERSAL_CONSTANT. The universal constant bound can be calculated
    // by the cubic root of the maximum number that the patch index type can hold.
    // In this case, with a 64  bits index we have a universal constant equal to (2^64)^(1.0/3.0).
    // The constant is rounded to the nearest lower even number for practical reasons.
    static const uint64_t UNIVERSAL_CONSTANT = 2642244;

    // Resolution of the map.
    const double resolution;
    // The scale of the map, i.e. the inverse of the resolution.
    const double scale;
    // The memory size of each individual cell
    const size_t cell_memory_size;

    // The length of the patch in cell units.
    const uint32_t patch_length;
    // The number of cells in the patch volume.
    const uint32_t patch_volume;

    // Less memory can be used when the map is used for 2d purposes.
    const bool is_3d;

    // Patches are kept on a sparse map and referenced by their unique id.
    // The container is wrapper around a copy-on-write structure so that we
    // share data efficiently, for example, duplicating a map only duplicates
    // patches that are accessed for writing during their lifetime.
    Dictionary<uint64_t, COWPtr< Container > > patches;

    virtual ~Map();

    /**
     * World to map coordinates.
     *
     * Convert continuous coordinates to discrete coordinates.
     * Because the coordinates are being discretized resolution is lost.
     * For example, in a map with 0.05 meters of resolution, the discrete
     * coordinates for (0, 0, 0) are the same for the coordinates (0.01, 0.02, 0.01).
     *
     * @param[in] coordinates Continous coordinates to be converted.
     *
     * @returns The corresponding discrete coordinates.
     */
    inline Vector3ui w2m(const Vector3d& coordinates) const
    { return ((tf_ * coordinates).array() + 0.5).cast<uint32_t>(); }

    /**
     * World to map coordinates.
     *
     * Convert continuous coordinates to discrete coordinates without descretizing.
     *
     * @param[in] coordinates Continous coordinates to be converted.
     *
     * @returns The corresponding discrete coordinates.
     */
    inline Vector3d w2m_nocast(const Vector3d& coordinates) const
    { return tf_ * coordinates; }

    /**
     * Convert discrete coordinates to continuous coordinates.
     *
     * @param[in] coordinates Discrete coordinates to be converted.
     *
     * @returns The corresponding continuous coordinates.
     */
    inline Vector3d m2w(const Vector3ui& coordinates) const
    { return tf_inv_ * coordinates.cast<double>(); }

    /**
     * Get the size of allocated memory.
     *
     * It only counts the memory allocated for the patches.
     *
     * @returns The total size of memory used by the map.
     */
    size_t memory() const;
    size_t fullMemory() const;

    inline size_t numOfPatches() const
    { return patches.size(); }

    /**
     * Calculate the metric bounds of the map in map coordinates.
     *
     * @param[out] min The lowest coordinates in the map.
     * @param[out] max The highest coordinates in the map.
     */
    void bounds(Vector3ui& min, Vector3ui& max) const;

    inline void bounds(Vector3d& min, Vector3d& max) const
    {
        Vector3ui m, M; bounds(m, M);
        min = m2w(m); max = m2w(M);
    }

    /**
     * Verify if the map has a patch allocated at the given coordinates.
     *
     * @param[in] coordinates Check for a patch here.
     *
     * @returns true if a patch is allocated, false otherwise.
     */
    bool patchAllocated(const Vector3ui& coordinates) const;
    bool patchIsUnique(const Vector3ui& coordinates) const;

    void useCompression(bool compression, uint32_t lru_size = 50,
                        const std::string& algorithm = "lz4");

    uint64_t hash(const Vector3ui& coordinates) const;

    /**
     * Write map to a file.
     *
     * @param[in] filename Output filename.
     *
     * @returns true if successfull, false otherwise.
     */
    bool write(const std::string& filename) const;

    /**
     * Read map from a file.
     *
     * @param[in] filename Input filename.
     *
     * @returns true if successfull, false otherwise.
     */
    bool read(const std::string& filename);

    inline uint32_t cacheHit() const
    { return cache_hit_; }

    inline uint32_t cacheMiss() const
    { return cache_miss_; }

    void computeRay(const Vector3ui& from, const Vector3ui& to, VectorVector3ui& sink);
    void computeRay(const Vector3d& from, const Vector3d& to, VectorVector3ui& sink);


    /// A cell walker is a function that is called with the
    /// map coordinates of an existing cell.
    typedef std::function<void(const Vector3ui&)> CellWalker;

    /// The method is more simple and cleaner than implementing
    /// and using an iterator. For each cells it calls a walker.
    /// Lambdas are great for this.
    void visit_all_cells(const CellWalker& walker);
    void visit_all_cells(const CellWalker& walker) const;

protected:

    /**
     * Default constructor.
     *
     * @param[in] resolution The granularity of the map.
     * @param[in] cell_size  The memory size of a single cell.
     * @param[in] patch_size The size of the faces of the patch.
     * @param[in] is3d       Should the map represent a 3D environment.
     */
    Map(double resolution, size_t cell_size, uint32_t patch_size, bool is3d);

    /**
     * Copy constructor.
     */
    Map(const Map& other);

    /**
     * Get a pointer to the cell located at @p coordinates.
     *
     * If the cell does not exist, i.e. a patch with a cell at @p coordinates
     * does not exist, a new patch is allocated. With this behavior, the map
     * can grow dynamically.
     *
     * @param[in] coordinates Location of the cells in map coordinates.
     *
     * @returns A pointer to the desired cell.
     */
    uint8_t* get(const Vector3ui& coordinates);

    /**
     * Get a constant pointer to the cell located at @p coordinates.
     *
     * If the cell does not exist, i.e. a patch with a cell at @p coordinates
     * does not exist, a new patch is allocated. With this behavior, the map
     * can grow dynamically.
     *
     * @param[in] coordinates Location of the cells in map coordinates.
     *
     * @returns A constant pointer to the desired cell.
     */
    const uint8_t* get(const Vector3ui& coordinates) const;


    /**
     * Write internal parameters of the map.
     */
    virtual void writeParameters(std::ofstream& stream) const
    {}

    /**
     * Read internal parameters of the map.
     */
    virtual void readParameters(std::ifstream& stream)
    {}


private:

    //Vector3ui unhash(uint64_t idx) const;
    Vector3ui unhash(uint64_t idx, uint64_t stride = UNIVERSAL_CONSTANT) const;

    bool lru_key_exists(uint64_t idx) const;
    void lru_put(uint64_t idx, COWPtr< Container >* container) const;

    COWPtr< Container >* lru_get(uint64_t idx) const;

private:

    Affine3d tf_;
    Affine3d tf_inv_;

    // LRU stuff
    typedef COWPtr< Container >* lru_type_t;
    typedef std::pair<uint64_t, lru_type_t> key_value_pair_t;
    typedef LinkedList<key_value_pair_t>::iterator list_iterator_t;

    mutable LinkedList<key_value_pair_t>          lru_items_list_;
    mutable Dictionary<uint64_t, list_iterator_t> lru_items_map_;

    char* buffer_;
    size_t lru_max_size_;
    bool use_compression_;
    mutable uint32_t cache_miss_;
    mutable uint32_t cache_hit_;

    ldc::BufferCompressor* bc_;
};

}} /* rtk::sdm */


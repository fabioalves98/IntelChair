//
// Copyright (c) 2018, IRIS Laboratory, IEETA, University of Aveiro - Portugal
//
// @date    2018-10-30
// @authors Eurico Pedrosa <efp@ua.pt>
//

#pragma once

#include <fstream>

#include "rtk/sdm/map.h"

namespace rtk {
namespace sdm {

/**
 * Calculates the distance to the closest occupied cells in the map.
 */
class DistanceMap : public Map {
public:

    DistanceMap(double resolution, size_t cell_size, uint32_t patch_size, bool is3d)
        : Map(resolution, cell_size, patch_size, is3d)
    {}

    DistanceMap(const DistanceMap& other)
        : Map(other)
    {}

    virtual ~DistanceMap()
    {}

    /**
     * Get the closest distance from the given coordinates.
     *
     * @param[in]  coordinates  Query coordinates.
     * @param[out] gradient     The gradient (or partial derivatives) at @p coordinates.
     *
     * @returns The distance to the closest occupied cell.
     */
    virtual double distance(const Vector3d& coordinates, Vector3d* gradient) const = 0;
    virtual double distance(const Vector3ui& coordinates) const = 0;

    virtual void setMaxDistance(double distance) = 0;
    virtual double maxDistance() const = 0;
};

}} /* rtk::sdm  */


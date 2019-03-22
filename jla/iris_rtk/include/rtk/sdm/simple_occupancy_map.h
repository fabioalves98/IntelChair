//
// Copyright (c) 2018, IRIS Laboratory, IEETA, University of Aveiro - Portugal
//
// @date    2018-10-30
// @authors Eurico Pedrosa <efp@ua.pt>
//

#pragma once

#include <rtk/sdm/occupancy_map.h>

namespace rtk {
namespace sdm {

class SimpleOccupancyMap : public OccupancyMap {
public:

    SimpleOccupancyMap(double resolution, uint32_t patch_size = 20, bool is3d = false);
    SimpleOccupancyMap(const SimpleOccupancyMap& other);

    virtual ~SimpleOccupancyMap();

    bool setFree(const Vector3d& coordinates);
    bool setFree(const Vector3ui& coordinates);

    bool setOccupied(const Vector3d& coordinates);
    bool setOccupied(const Vector3ui& coordinates);

    bool setUnknown(const Vector3d& coordinates);
    bool setUnknown(const Vector3ui& coordinates);

    bool isFree(const Vector3d& coordinates) const;
    bool isFree(const Vector3ui& coordinates) const;

    bool isOccupied(const Vector3d& coordinates) const;
    bool isOccupied(const Vector3ui& coordinates) const;

    bool isUnknown(const Vector3d& coordinates) const;
    bool isUnknown(const Vector3ui& coordinates) const;

    double getProbability(const Vector3d& coordinates) const;
    double getProbability(const Vector3ui& coordinates) const;

protected:

    /**
     * Write internal parameters of the map.
     */
    void writeParameters(std::ofstream& stream) const
    {}

    /**
     * Read internal parameters of the map.
     */
    void readParameters(std::ifstream& stream)
    {}

private:
    SimpleOccupancyMap& operator=(const SimpleOccupancyMap& other);
};

}} /* rtk::sdm */


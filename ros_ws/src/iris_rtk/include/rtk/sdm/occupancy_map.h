//
// Copyright (c) 2018, IRIS Laboratory, IEETA, University of Aveiro - Portugal
//
// @date    2018-10-30
// @authors Eurico Pedrosa <efp@ua.pt>
//

#pragma once

#include <string>
#include <limits>

#include <rtk/sdm/map.h>

namespace rtk {
namespace sdm {

class OccupancyMap : public Map {
public:

    OccupancyMap(double resolution, size_t cell_size, uint32_t patch_size, bool is3d)
        : Map(resolution, cell_size, patch_size, is3d)
    {}

    OccupancyMap(const OccupancyMap& other)
        : Map(other)
    {}

    virtual ~OccupancyMap()
    {}

    virtual bool setFree(const Vector3d& coordinates) = 0;
    virtual bool setFree(const Vector3ui& coordinates) = 0;

    virtual bool setOccupied(const Vector3d& coordinates) = 0;
    virtual bool setOccupied(const Vector3ui& coordinates) = 0;

    virtual bool setUnknown(const Vector3d& coordinates) = 0;
    virtual bool setUnknown(const Vector3ui& coordinates) = 0;

    virtual bool isFree(const Vector3d& coordinates) const = 0;
    virtual bool isFree(const Vector3ui& coordinates) const = 0;

    virtual bool isOccupied(const Vector3d& coordinates) const = 0;
    virtual bool isOccupied(const Vector3ui& coordinates) const = 0;

    virtual bool isUnknown(const Vector3d& coordinates) const = 0;
    virtual bool isUnknown(const Vector3ui& coordinates) const = 0;

    virtual double getProbability(const Vector3d& coordinates) const = 0;
    virtual double getProbability(const Vector3ui& coordinates) const = 0;

    //size_t insertPointCloud(const PointCloudXYZ::Ptr& cloud, bool fast = false);
};

}} /* rtk::sdm */


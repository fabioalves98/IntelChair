//
// Copyright (c) 2018, IRIS Laboratory, IEETA, University of Aveiro - Portugal
//
// @date    2018-10-30
// @authors Eurico Pedrosa <efp@ua.pt>
//

#include <rtk/sdm/simple_occupancy_map.h>

namespace rtk {
namespace sdm {

SimpleOccupancyMap::SimpleOccupancyMap(double resolution, uint32_t patch_size, bool is3d)
    : OccupancyMap(resolution, sizeof(char), patch_size, is3d)
{}

SimpleOccupancyMap::SimpleOccupancyMap(const SimpleOccupancyMap& other)
    : OccupancyMap(other)
{}

SimpleOccupancyMap::~SimpleOccupancyMap()
{}

bool SimpleOccupancyMap::setFree(const Vector3d& coordinates)
{
    return setFree(w2m(coordinates));
}

bool SimpleOccupancyMap::setFree(const Vector3ui& coordinates)
{
    char* cell = (char*) get(coordinates);
    if (*cell == -1)
        return false;

    *cell = -1;
    return true;
}

bool SimpleOccupancyMap::setOccupied(const Vector3d& coordinates)
{
    return setOccupied(w2m(coordinates));
}

bool SimpleOccupancyMap::setOccupied(const Vector3ui& coordinates)
{
    char* cell = (char*) get(coordinates);
    if (*cell == 1)
        return false;

    *cell = 1;
    return true;
}

bool SimpleOccupancyMap::setUnknown(const Vector3d& coordinates)
{
    return setUnknown(w2m(coordinates));
}

bool SimpleOccupancyMap::setUnknown(const Vector3ui& coordinates)
{
    char* cell = (char*) get(coordinates);
    if ( *cell == 0 )
        return false;

    *cell = 0;
    return true;
}

bool SimpleOccupancyMap::isFree(const Vector3d& coordinates) const
{
    return isFree(w2m(coordinates));
}

bool SimpleOccupancyMap::isFree(const Vector3ui& coordinates) const
{
    const char* cell = (const char*) get(coordinates);
    if (cell == 0)
        return false;

    return *cell == -1;
}

bool SimpleOccupancyMap::isOccupied(const Vector3d& coordinates) const
{
    return isOccupied(w2m(coordinates));
}

bool SimpleOccupancyMap::isOccupied(const Vector3ui& coordinates) const
{
    const char* cell = (const char*) get(coordinates);
    if (cell == 0)
        return false;

    return *cell == 1;
}

bool SimpleOccupancyMap::isUnknown(const Vector3d& coordinates) const
{
    return isUnknown(w2m(coordinates));
}

bool SimpleOccupancyMap::isUnknown(const Vector3ui& coordinates) const
{
    const char* cell = (const char*) get(coordinates);
    if (cell == 0) return true;

    return *cell == 0;
}

double SimpleOccupancyMap::getProbability(const Vector3d& coordinates) const
{
    return getProbability(w2m(coordinates));
}

double SimpleOccupancyMap::getProbability(const Vector3ui& coordinates) const
{
    const char* cell = (const char*) get(coordinates);
    if (cell == 0) return 0.5;

    switch(*cell){
        case -1: return 0.0;
        case  1: return 1.0;
        default: return 0.5;
    }
}

}} /* rtk::sdm */


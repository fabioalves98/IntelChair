//
// Copyright (c) 2018, IRIS Laboratory, IEETA, University of Aveiro - Portugal
//
// @date    2018-10-30
// @authors Eurico Pedrosa <efp@ua.pt>
//

#include <rtk/sdm/frequency_occupancy_map.h>

namespace rtk {
namespace sdm {

static double occ_thresh = 0.25;

static double prob(const FrequencyOccupancyMap::frequency& freq)
{
    if (freq.visited == 0) return occ_thresh;
    return ((double)freq.occupied) / ((double)freq.visited);
    //return ((double)freq.occupied) / ((double)(freq.visited+freq.occupied));
}

FrequencyOccupancyMap::FrequencyOccupancyMap(double resolution, uint32_t patch_size, bool is3d)
    : OccupancyMap(resolution, sizeof(frequency), patch_size, is3d)
{}

FrequencyOccupancyMap::FrequencyOccupancyMap(const FrequencyOccupancyMap& other)
    : OccupancyMap(other)
{}

FrequencyOccupancyMap::~FrequencyOccupancyMap()
{}

bool FrequencyOccupancyMap::setFree(const Vector3d& coordinates)
{
    return setFree(w2m(coordinates));
}

bool FrequencyOccupancyMap::setFree(const Vector3ui& coordinates)
{
    frequency* cell = (frequency*) get(coordinates);

    bool free = prob(*cell) < occ_thresh;
    cell->visited++;

    if ( free ) return false;
    else return (prob(*cell) < occ_thresh);
}

bool FrequencyOccupancyMap::setOccupied(const Vector3d& coordinates)
{
    return setOccupied(w2m(coordinates));
}

bool FrequencyOccupancyMap::setOccupied(const Vector3ui& coordinates)
{
    frequency* cell = (frequency*) get(coordinates);

    bool occupied = prob(*cell) > occ_thresh;
    cell->occupied++;
    cell->visited++;

    if ( occupied ) return false;
    else return (prob(*cell) > occ_thresh);
}

bool FrequencyOccupancyMap::setUnknown(const Vector3d& coordinates)
{
    return setUnknown(w2m(coordinates));
}

bool FrequencyOccupancyMap::setUnknown(const Vector3ui& coordinates)
{
    frequency* cell = (frequency*) get(coordinates);

    if ( cell->visited == 0 ) return false;

    cell->occupied = 0;
    cell->visited  = 0;

    return true;
}

bool FrequencyOccupancyMap::isFree(const Vector3d& coordinates) const
{
    return isFree(w2m(coordinates));
}

bool FrequencyOccupancyMap::isFree(const Vector3ui& coordinates) const
{
    const frequency* cell = (const frequency*) get(coordinates);
    if ( cell == 0 ) return false;

    return prob(*cell) < occ_thresh;
}

bool FrequencyOccupancyMap::isOccupied(const Vector3d& coordinates) const
{
    return isOccupied(w2m(coordinates));
}

bool FrequencyOccupancyMap::isOccupied(const Vector3ui& coordinates) const
{
    const frequency* cell = (const frequency*) get(coordinates);
    if ( cell == 0 ) return false;

    return prob(*cell) > occ_thresh;
}

bool FrequencyOccupancyMap::isUnknown(const Vector3d& coordinates) const
{
    return isUnknown(w2m(coordinates));
}

bool FrequencyOccupancyMap::isUnknown(const Vector3ui& coordinates) const
{
    const frequency* cell = (const frequency*) get(coordinates);
    if (cell == 0) return true;

    return cell->visited == 0;
}

double FrequencyOccupancyMap::getProbability(const Vector3d& coordinates) const
{
    return getProbability(w2m(coordinates));
}

double FrequencyOccupancyMap::getProbability(const Vector3ui& coordinates) const
{
    const frequency* cell = (const frequency*) get(coordinates);
    if (cell == 0) return occ_thresh;

    return prob(*cell);
}

}} /* rtk::sdm */

//
// Copyright (c) 2018, IRIS Laboratory, IEETA, University of Aveiro - Portugal
//
// @date    2018-10-30
// @authors Eurico Pedrosa <efp@ua.pt>
//

#include <rtk/sdm/probabilistic_occupancy_map.h>

namespace rtk {
namespace sdm {

inline float prob(const float& logods)
{
    return 1.0 - 1.0 / (1.0 + std::exp(logods));
}

inline float logods(const float& prob)
{
    return std::log(prob / (1.0 - prob));
}

ProbabilisticOccupancyMap::ProbabilisticOccupancyMap(double resolution, uint32_t patch_size, bool is3d)
    : OccupancyMap(resolution, sizeof(prob_tag), patch_size, is3d)
{
    miss_ = logods(0.4);
    hit_  = logods(0.7);

    clamp_min_ = logods(0.12);
    clamp_max_ = logods(0.97);

    occ_thresh_ = 0.0 *logods(0.5);
}

ProbabilisticOccupancyMap::ProbabilisticOccupancyMap(const ProbabilisticOccupancyMap& other)
    : OccupancyMap(other)
{
    miss_ = other.miss_;
    hit_  = other.hit_;

    clamp_min_ = other.clamp_min_;
    clamp_max_ = other.clamp_max_;

    occ_thresh_ = other.occ_thresh_;
}

ProbabilisticOccupancyMap::~ProbabilisticOccupancyMap()
{}

bool ProbabilisticOccupancyMap::setFree(const Vector3d& coordinates)
{
    return setFree(w2m(coordinates));
}

bool ProbabilisticOccupancyMap::setFree(const Vector3ui& coordinates)
{
    prob_tag* cell = (prob_tag*) get(coordinates);

    bool free = cell->prob < occ_thresh_;
    cell->prob = std::max(cell->prob + miss_, clamp_min_);

    if ( free ) return false;
    else return (cell->prob < occ_thresh_);
}

bool ProbabilisticOccupancyMap::setOccupied(const Vector3d& coordinates)
{
    return setOccupied(w2m(coordinates));
}

bool ProbabilisticOccupancyMap::setOccupied(const Vector3ui& coordinates)
{
    prob_tag* cell = (prob_tag*) get(coordinates);

    bool occupied = cell->prob > occ_thresh_;
    cell->prob = std::min(cell->prob + hit_, clamp_max_);

    if ( occupied ) return false;
    else return (cell->prob > occ_thresh_);
}

bool ProbabilisticOccupancyMap::setUnknown(const Vector3d& coordinates)
{
    return setUnknown(w2m(coordinates));
}

bool ProbabilisticOccupancyMap::setUnknown(const Vector3ui& coordinates)
{
    prob_tag* cell = (prob_tag*) get(coordinates);

    bool unknown = cell->prob == occ_thresh_;
    cell->prob = occ_thresh_;

    if ( unknown ) return false;
    else return true;
}

bool ProbabilisticOccupancyMap::isFree(const Vector3d& coordinates) const
{
    return isFree(w2m(coordinates));
}

bool ProbabilisticOccupancyMap::isFree(const Vector3ui& coordinates) const
{
    const prob_tag* cell = (const prob_tag*) get(coordinates);
    if (cell == 0) return false;

    return cell->prob < occ_thresh_;
}

bool ProbabilisticOccupancyMap::isOccupied(const Vector3d& coordinates) const
{
    return isOccupied(w2m(coordinates));
}

bool ProbabilisticOccupancyMap::isOccupied(const Vector3ui& coordinates) const
{
    const prob_tag* cell = (const prob_tag*) get(coordinates);
    if (cell == 0) return false;

    return cell->prob > occ_thresh_;
}

bool ProbabilisticOccupancyMap::isUnknown(const Vector3d& coordinates) const
{
    return isUnknown(w2m(coordinates));
}

bool ProbabilisticOccupancyMap::isUnknown(const Vector3ui& coordinates) const
{
    const prob_tag* cell = (const prob_tag*) get(coordinates);
    if (cell == 0) return true;

    return cell->prob == occ_thresh_;
}

double ProbabilisticOccupancyMap::getProbability(const Vector3d& coordinates) const
{
    return getProbability(w2m(coordinates));
}

double ProbabilisticOccupancyMap::getProbability(const Vector3ui& coordinates) const
{
    const prob_tag* cell = (const prob_tag*) get(coordinates);
    if (cell == 0) return prob(occ_thresh_);

    return prob(cell->prob);
}

}} /* rtk::sdm */

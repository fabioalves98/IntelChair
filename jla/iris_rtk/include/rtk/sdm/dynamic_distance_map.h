//
// Copyright (c) 2018, IRIS Laboratory, IEETA, University of Aveiro - Portugal
//
// @date    2018-10-30
// @authors Eurico Pedrosa <efp@ua.pt>
//

#pragma once

#include <rtk/structs/priority_queue.h>

#include <rtk/sdm/distance_map.h>

namespace rtk {
namespace sdm {

/**
 * Dynamic version of a distance map.
 */
class DynamicDistanceMap : public DistanceMap {
public:

    struct distance_t {
        Vector3ui obstacle;
        uint16_t sqdist;
        bool valid_obstacle;
        bool is_queued;
    };

    DynamicDistanceMap(const DynamicDistanceMap& other);

    DynamicDistanceMap(double resolution, uint32_t patch_size = 20, bool is3d = false);
    virtual ~DynamicDistanceMap();

    double distance(const Vector3d& coordinates, Vector3d* gradient = 0) const;
    double distance(const Vector3ui& coordinates) const;

    void addObstacle(const Vector3ui& location);
    void removeObstacle(const Vector3ui& location);

    uint32_t update();

    void setMaxDistance(double distance);
    double maxDistance() const;

protected:

    /**
     * Write internal parameters of the map.
     */
    void writeParameters(std::ofstream& stream) const;

    /**
     * Read internal parameters of the map.
     */
    void readParameters(std::ifstream& stream);

private:

    void raise(const Vector3ui& location, distance_t& current);
    void lower(const Vector3ui& location, distance_t& current);

private:

    PriorityQueue<Vector3ui> lower_;
    PriorityQueue<Vector3ui> raise_;

    int deltas_[26][3];
    uint32_t max_sqdist_;

};

}} /* rtk::sdm */


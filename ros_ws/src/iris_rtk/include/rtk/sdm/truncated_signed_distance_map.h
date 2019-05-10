//
// Copyright (c) 2018, IRIS Laboratory, IEETA, University of Aveiro - Portugal
//
// @date    2018-10-30
// @authors Eurico Pedrosa <efp@ua.pt>
//

#pragma once

#include <rtk/sdm/distance_map.h>
#include <rtk/structs/priority_queue.h>

namespace rtk {
namespace sdm {


/**
 * Truncated Signed Distance Map.
 */
class TruncatedSignedDistanceMap : public DistanceMap {
public:

    struct truncated_signed_distance_t {
        float distance;
        float weight;
    };

    TruncatedSignedDistanceMap(const TruncatedSignedDistanceMap& other);
    TruncatedSignedDistanceMap(double resolution, uint32_t patch_size = 16, bool is3d = false);
    virtual ~TruncatedSignedDistanceMap();

    double distance(const Vector3d& coordinates, Vector3d* gradient = 0) const;
    double distance(const Vector3ui& coordinates) const;

    size_t insertPointCloud(const PointCloudXYZ::Ptr& cloud);
    void integrate(const Vector3d& origin, const Vector3d& hit);

    void setMaxDistance(double distance);
    double maxDistance() const;

private:

    float maximum_distance_;
    float maximum_weight_;
    float truncate_size_;

    float epsilon_;
    float delta_;

};

}} /* rtk::sdm */



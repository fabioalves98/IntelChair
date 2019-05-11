//
// Copyright (c) 2018, IRIS Laboratory, IEETA, University of Aveiro - Portugal
//
// @date    2018-10-30
// @authors Eurico Pedrosa <efp@ua.pt>
//

#pragma once

#include <stdint.h>

#include <memory>
#include <vector>
#include <deque>
#include <list>
#include <map>
#include <set>

#include <unordered_set>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace Eigen {

typedef Matrix<uint32_t, 3, 1> Vector3ui;

typedef Matrix<uint64_t, 3, 1> Vector3ul;
typedef Matrix<int64_t, 3, 1>  Vector3l;

typedef std::deque<Vector3d, aligned_allocator<Vector3d> > VectorVector3d;
typedef std::deque<Vector3ui, aligned_allocator<Vector3ui> > VectorVector3ui;

typedef std::deque<VectorXd, aligned_allocator<VectorXd> > VectorVectorXd;

}

namespace rtk {

template<class T>
using List = std::deque<T>;

template<class T>
using LinkedList = std::list<T>;

// A vector is an algebraic construction, not a container.
// What this container actually represents is an array
// with a continuous memory segment with dynamic size.
template<class T>
using DynamicArray = std::vector<T>;

template<class Key, class T>
using Dictionary = std::map<Key, T>;

//--

using namespace Eigen;

template <typename T> int sign(T val) {
    return (T(0) < val) - (val < T(0));
}

struct KeyHash {
    size_t operator()(const Vector3ui& key) const
    { return key(2) + 2642244ul * ( key(1) + key(0) * 2642244ul); }
};
typedef std::unordered_set<Vector3ui, KeyHash> KeySet;


struct PointCloudXYZ {
    typedef std::shared_ptr<PointCloudXYZ> Ptr;

    std::vector<Vector3d> points;

    Vector3d    sensor_origin_;
    Quaterniond sensor_orientation_;
};

}


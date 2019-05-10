//
// Copyright (c) 2018, IRIS Laboratory, IEETA, University of Aveiro - Portugal
//
// @date    2018-10-30
// @authors Eurico Pedrosa <efp@ua.pt>
//

#pragma once

namespace rtk {
namespace sdm {

struct OccupancyMap;
struct DistanceMap;

bool export_to_png(const OccupancyMap& occ, const std::string& filename, double zed = 0.0);
bool export_to_png(const DistanceMap&   dm, const std::string& filename, double zed = 0.0);

}} // namespace rtk::sdm


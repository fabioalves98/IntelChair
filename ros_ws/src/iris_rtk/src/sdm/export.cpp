//
// Copyright (c) 2018, IRIS Laboratory, IEETA, University of Aveiro - Portugal
//
// @date    2018-10-30
// @authors Eurico Pedrosa <efp@ua.pt>
//

#include "rtk/structs/image.h"
#include "rtk/io/image_io.h"

#include "rtk/sdm/occupancy_map.h"
#include "rtk/sdm/distance_map.h"
#include "rtk/sdm/export.h"

namespace {

void build_image(const rtk::sdm::OccupancyMap& occ, rtk::Image& image, double zed)
{
    rtk::Vector3ui min, max;
    occ.bounds(min,max);

    rtk::Vector3ui dim = max - min;

    min(2) = occ.w2m(rtk::Vector3d(0,0,zed)).z();

    image.alloc(dim(0), dim(1), 1);
    image.fill(90);

    occ.visit_all_cells([&image, &occ, &min](const rtk::Vector3ui& coords){
        // Filter by z (zed).
        if (occ.is_3d and coords(2) != min(2))
            return;

        rtk::Vector3ui adj_coords = coords - min;

        if (occ.isFree(coords))
            image(adj_coords(0), adj_coords(1)) = 255;
        else if (occ.isOccupied(coords))
            image(adj_coords(0), adj_coords(1)) = 0;
        else
            image(adj_coords(0), adj_coords(1)) = 127;
    });
}

void build_image(const rtk::sdm::DistanceMap& dm, rtk::Image& image, double zed)
{
    rtk::Vector3ui min, max;
    dm.bounds(min,max);

    rtk::Vector3ui dim = max - min;

    min(2) = dm.w2m(rtk::Vector3d(0,0,zed)).z();

    image.alloc(dim(0), dim(1), 1);
    image.fill(127);

    dm.visit_all_cells([&image, &dm, &min](const rtk::Vector3ui& coords){
        // Filter by z_slice.
        if (dm.is_3d and coords(2) != min(2))
            return;

        rtk::Vector3ui adj_coords = coords - min;
        image(adj_coords(0), adj_coords(1)) = dm.distance(coords) * 255 / dm.maxDistance();
    });
}

} // namespace

bool rtk::sdm::export_to_png(const OccupancyMap& occ, const std::string& filename, double zed)
{
    Image image;
    build_image(occ, image, zed);
    return io::image_write_png(image, filename);
}

bool rtk::sdm::export_to_png(const DistanceMap& dm, const std::string& filename, double zed)
{
    Image image;
    build_image(dm, image, zed);
    return io::image_write_png(image, filename);
}


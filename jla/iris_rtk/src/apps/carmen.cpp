//
// Copyright (c) 2018, IRIS Laboratory, IEETA, University of Aveiro - Portugal
//
// @date    2018-10-30
// @authors Eurico Pedrosa <efp@ua.pt>
//

#include <algorithm>

#include "rtk/print.h"

#include <rtk/geom/pose3d.h>

#include "carmen.h"

namespace rtk {
namespace dataset {

using namespace std;

CarmenReader::CarmenReader()
{
    sensor_origin_.setZero();
}

CarmenReader::~CarmenReader()
{
    close();
}

bool CarmenReader::open(const std::string& logname)
{
    in_.open(logname.c_str());
    if (not in_.is_open())
        return false;

    return true;
}

void CarmenReader::close()
{
    in_.close();
}


bool CarmenReader::readLaserScan(LaserScan* ls)
{

    while(in_){
        std::string line;
        std::getline(in_, line);
        std::stringstream ss(line);

        std::string type;
        ss >> type;

        std::transform(type.begin(), type.end(), type.begin(), ::tolower);

        bool ok;
        if (type == "param"){
            parseParam(ss);
            // continue until a laser is found.
        }else if (type == "flaser"){

            ok = parseFLaser(ss, ls);
            if (ok) return true;

        }else if (type == "robotlaser1"){

            ok = parseRobotLaser(ss, ls);
            if (ok) return true;

        }
    }// end while

    return false;
}


void CarmenReader::parseParam(std::stringstream& ss)
{
    std::string param;
    ss >> param;

    if (param == "robot_frontlaser_offset"){
        double offset;
        ss >> offset;
        sensor_origin_[0] = offset;
        sensor_origin_[1] = 0.0;
        sensor_origin_[2] = 0.0;
    }

}

bool CarmenReader::parseFLaser(std::stringstream& ss, LaserScan* ls)
{
    uint32_t num_readings;
    ss >> num_readings;

    ls->ranges.resize(num_readings);

    ls->range_max = 50;
    ls->range_min = 0.1;

    // The laser-scan has a fov of 180º in the interval [-pi/2, pi/2]
    ls->angle_min = -M_PI * 0.5;
    ls->angle_max =  M_PI * 0.5;
    ls->angle_inc =  M_PI / num_readings;

    ls->ranges.resize(num_readings);
    for (uint32_t i = 0; i < num_readings; ++i)
        ss >> ls->ranges[i];

    double x, y, yaw;
    ss >> x >> y >> yaw; // this is the laser position.
    ss >> x >> y >> yaw; // this is the odometry
    ls->origin   = geom::Pose3D(sensor_origin_, 0);
    ls->location = geom::Pose3D(Vector3d(x, y, 0.0), yaw);

    ss >> ls->stamp;

    return true;
}

bool CarmenReader::parseRobotLaser(std::stringstream& ss, LaserScan* ls)
{
    string dummy;
    int numOfReading;

    ss >> dummy >> ls->angle_min >> dummy >> ls->angle_inc
       >> ls->range_max >> dummy >> dummy >> numOfReading;

    ls->range_min = 0.01;

    ls->ranges.resize(numOfReading);
    for (int i = 0; i < numOfReading; ++i)
        ss >> ls->ranges[i];

    // skip remission
    ss >> numOfReading;
    for (int i = 0; i < numOfReading; ++i)
        ss >> dummy;

    double x, y, yaw;
    ss >> x >> y >> yaw; // this is the laser position, and is to be ignored.
    ss >> x >> y >> yaw; // this is the odometry

    ls->location = geom::Pose3D(Vector3d(x, y, 0.0), yaw);

    ss >> dummy >> dummy >> dummy >> dummy >> dummy;
    ss >> ls->stamp;

    return true;
}

//==================================================================================================

CarmenWriter::CarmenWriter()
{}

CarmenWriter::~CarmenWriter()
{
    close();
}

bool CarmenWriter::open(const std::string& logname)
{
    out_.open(logname.c_str());
    if (not out_.is_open())
        return false;

    return true;
}

void CarmenWriter::close()
{
    out_.close();
}

bool CarmenWriter::writeLaserScan(const LaserScan& ls)
{
    if (ls.ranges.size() < 2){
        // use the old FLASER
        auto message = format("FLASER 0 0 0 0 %g %g %g %.6f slam %.6f",
                              ls.origin.x(), ls.origin.y(), ls.origin.yaw(),
                              ls.stamp, ls.stamp);

        out_ << message << std::endl;
        return true;
    }

    return false;

    // TODO:
    // ROBOTLASER1 laser_type start_angle field_of_view angular_resolution
    // maximum_range accuracy remission_mode
    // num_readings [range_readings] laser_pose_x laser_pose_y laser_pose_theta
    // robot_pose_x robot_pose_y robot_pose_theta
    // laser_tv laser_rv forward_safety_dist side_safty_dist

    return true;
}

bool CarmenWriter::writePose(const geom::Pose2D& pose, double stamp)
{
    // use the old FLASER
    auto message = format("FLASER 0 0 0 0 %g %g %g %.6f slam %.6f",
                          pose.x(), pose.y(), pose.rotation(),
                          stamp, stamp);

    out_ << message << std::endl;
    return true;
}

}} /* rtk::dataset */


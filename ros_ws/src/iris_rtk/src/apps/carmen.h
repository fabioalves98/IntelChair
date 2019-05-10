//
// Copyright (c) 2018, IRIS Laboratory, IEETA, University of Aveiro - Portugal
//
// @date    2018-10-30
// @authors Eurico Pedrosa <efp@ua.pt>
//

#pragma once

#include <string>
#include <fstream>
#include <sstream>

#include <rtk/structs/laser_scan.h>
#include <rtk/geom/pose2d.h>

namespace rtk      {
namespace dataset {

class CarmenReader {
public:

    CarmenReader();
    ~CarmenReader();

    /**
     * Open a carmen log file for reading.
     *
     * This will only open the file for reading, no data
     * will be parsed. To access the data inside the log
     * use readData().
     *
     * @param logname Name (or path) of the logfile.
     * @returns true if the file was open, false otherwise.
     *
     * @see readDataLaserScan()
     */
    bool open(const std::string& logname);

    /**
     * Close the file handler.
     */
    void close();

    /**
     * Retrieve a laser-scan from the log file.
     * Each laser scan has also odometry and timestamp information.
     *
     * @param ls        A laser-scan object that will hold the data.
     * @param odometry  The odometry dead reckoning time syncronized with the laser-scan.
     * @param timestap  The timestamp of the data.
     *
     * @returns true if a laser-scan was read, false otherwise.
     */
    bool readLaserScan(LaserScan* ls);

private:

    void parseParam(std::stringstream& ss);

    bool parseFLaser(std::stringstream& ss, LaserScan* ls);
    bool parseRobotLaser(std::stringstream& ss, LaserScan* ls);

private:
    std::ifstream in_;
    Vector3d sensor_origin_;
};

class CarmenWriter {
public:

    CarmenWriter();
    ~CarmenWriter();

    /**
     * Open a carmen log file for writing.
     **
     * @param logname Name (or path) of the logfile.
     * @returns true if the file was open, false otherwise.
     *
     */
    bool open(const std::string& logname);

    void close();

    template<typename T>
    void writeParam(const std::string& name, const T& value)
    {
        out_ << "PARAM " << name << " " << value << std::endl;
    }

    bool writePose(const geom::Pose2D& pose, double stamp);

    bool writeLaserScan(const LaserScan& ls);

private:
    std::ofstream out_;
};

}} /* tk::dataset */


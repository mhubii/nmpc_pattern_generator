#ifndef IO_MODULE_UTILS_H_
#define IO_MODULE_UTILS_H_

#include <yarp/os/PortReader.h>
#include <yarp/os/TypedReaderCallback.h>
#include <Eigen/Dense>

// Convert to radian.
#define DEG2RAD M_PI/180.0
#define RAD2DEG 180.0/M_PI

// Current status of the writer.
enum InitialPositionStatus { NOT_STARTED, MOVING, DONE };

// Part of the robot, e.g. the left leg.
struct Part {
    std::string name;
    std::vector<int> joints;
    std::vector<std::string> cameras;
};

#endif
#ifndef IO_MODULE_UTILS_H_
#define IO_MODULE_UTILS_H_

#include <yarp/os/PortReader.h>
#include <yarp/os/TypedReaderCallback.h>
#include <Eigen/Dense>

// Convert to radian.
#define DEG2RAD M_PI/180.0

// Current status of the pattern generate.
enum PatternGeneratorStatus { IDLE, PENDING };


// Part of the robot, e.g. the left leg.
struct Part {
    std::string name;
    std::vector<int> joints;
    std::vector<std::string> cameras;
};

#endif
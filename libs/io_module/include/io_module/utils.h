#ifndef IO_MODULE_UTILS_H_
#define IO_MODULE_UTILS_H_

#include <yarp/os/PortReader.h>
#include <yarp/os/TypedReaderCallback.h>
#include <Eigen/Dense>

// Convert to radian.
#define DEG2RAD M_PI/180.0
#define RAD2DEG 180.0/M_PI

// Status of the robot.
enum RobotStatus { NOT_CONNECTED, NOT_INITIALIZED, INITIALIZING, INITIALIZED, STOPPING };

// Errors and warnings.
enum Errors { NO_ERRORS, QP_INFEASIBLE, HARDWARE_LIMITS };
enum Warnings { NO_WARNINGS, IK_DID_NOT_CONVERGE };

// Part of the robot, e.g. the left leg.
struct Part {
    std::string name;
    std::vector<int> joints;
    std::vector<std::string> cameras;
};

#endif
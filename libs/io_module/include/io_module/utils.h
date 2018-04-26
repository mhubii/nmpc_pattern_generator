#ifndef IO_MODULE_UTILS_H_
#define IO_MODULE_UTILS_H_

#include <yarp/os/PortReader.h>
#include <yarp/os/TypedReaderCallback.h>
#include <Eigen/Dense>


// Current status of the pattern generate.
enum PatternGeneratorStatus { IDLE, PENDING };


// Part of the robot, e.g. the left leg.
struct Part {
    std::string name;
    std::vector<int> joints;
    std::vector<std::string> cameras;
};


// DepthProcessor.
class DepthProcessor : public yarp::os::TypedReaderCallback<yarp::os::Bottle>
{
    virtual void onRead(yarp::os::Bottle& bottle) {
        
    };
};

#endif
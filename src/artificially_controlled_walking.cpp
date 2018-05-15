#include <yarp/os/all.h>
#include <yarp/os/all.h>


// Forward declare WalkingProcessor. This is actually the heart
// of the application. Within it, the pattern is generated,
// and the dynamic filter as well as the inverse kinematics are
// computed.
class WalkingProcessor;


// Forward declare AIProcessor. This data processor takes depth
// images as computed from the stereo cameras of the robot and,
// based on them, decides which action to take.
class AIProcessor;


// Main application for heicub_walking.
int main() {

    // Read user input, mainly locations of configuration files.


    // Set up the yarp network.
    yarp::os::Network yarp;

    // 

    // Possible external input.

    // Rate threads.

    return 0;
}


// Implement WalkingProcessor.
class WalkingProcessor : public yarp::os::BufferedPort<yarp::os::Bottle>
{

};


// Implement AIProcessor.
class AIProcessor : public yarp::os::BufferedPort<yarp::os::Bottle>
{

};
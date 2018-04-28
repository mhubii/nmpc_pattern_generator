#include <yarp/os/Network.h>
#include <yarp/os/PortReader.h>

// Forward declare data processor. This is actually the heart
// of the application. Within it, the pattern is generated,
// as well as the daynamic filter and the inverse kinematics.
class DataProcessor;

// Main application for heicub_walking.
int main() {
    yarp::os::Network yarp;

    // Possible external input.

    // Rate threads.

    return 0;
}

class DataProcessor : public yarp::os::PortReader
{

};
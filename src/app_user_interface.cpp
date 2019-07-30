#include <yarp/os/all.h>

#include "reader.h"

int main(int argc, char** argv) {

    // Read in the mode.
    bool simulation = (!strcmp(argv[1], "y") || !strcmp(argv[1], "Y")) ? true : false;

    // Create yarp network.
    yarp::os::Network yarp;

    if (!yarp.checkNetwork()) {

        std::cout << "No yarpserver is running. Please run a yarpserver." << std::endl;
        std::exit(1);
    }

    // Create an app user interface.
    AppReader ar(simulation);

    // Open port for communication.
    ar.open("/user_interface/robot_status");

    // Start reading incomming commands.
    ar.ReadCommands();

    // Close connections.
    ar.close();

    return 0;
}
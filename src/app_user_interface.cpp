#include <yarp/os/all.h>

#include "reader.h"

int main() {

    // Create yarp network.
    yarp::os::Network yarp;

    if (!yarp.checkNetwork()) {

        std::cout << "No yarpserver is running. Please run a yarpserver." << std::endl;
        std::exit(1);
    }

    // Create an app user interface.
    AppReader ar;

    // Open port for communication.
    ar.open("/app_reader/commands");

    // Start reading incomming commands.
    ar.ReadCommands();

    // Close connections.
    ar.close();

    return 0;
}
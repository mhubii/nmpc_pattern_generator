#include <yarp/os/all.h>

#include "reader.h"

int main() {

    // Create yarp network.
    yarp::os::Network yarp;

    // Create a keyboard user interface.
    KeyReader kr;

    // Open port for communication.
    kr.open("/key_reader/commands");

    // Start reading incomming commands.
    kr.ReadCommands();

    // Close connections.
    kr.close();

    return 0;
}
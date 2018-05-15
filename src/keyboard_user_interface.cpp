#include <yarp/os/all.h>

#include "reader.h"

int main() {

    // Create yarp network.
    yarp::os::Network yarp;

    // Create a keyboard user interface.
    KeyReader kr;

    // Start reading incomming commands.
    kr.ReadCommands();

    return 0;
}
#include <yarp/os/all.h>

#include "reader.h"

int main(int argc, char** argv) {

    // Read in the mode.
    bool simulation = (!strcmp(argv[1], "y") || !strcmp(argv[1], "Y")) ? true : false;
    std::string mode = argv[2];

    if (!(!strcmp(mode.c_str(), "uc") || 
          !strcmp(mode.c_str(), "bc") || 
          !strcmp(mode.c_str(), "ba"))) {

        printf("Please provide a mode in which the interface shall be run. Modes are \n\
                uc = user conrolled\n\
                bc = behavioural cloning\n\
                ba = behavioural augmentation\n");
        std::exit(1);
    }

    // Create yarp network.
    yarp::os::Network yarp;

    // Create a keyboard user interface.
    KeyReader kr(simulation, mode);

    // Open port for communication.
    kr.open("/user_interface/robot_status");

    // Start reading incomming commands.
    kr.ReadCommands(); // program caught in while loop until q is pressed

    // Close connections.
    kr.close();

    return 0;
}

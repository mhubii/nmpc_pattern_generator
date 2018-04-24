#include <stdio.h>
#include <yarp/os/all.h>

#include "reader.h"

int main(int argc, char * argv[]) {
    yarp::os::Network yarp;

    KeyCommands key_commands;

    yarp::os::ResourceFinder rf;
    rf.configure(argc, argv);
    rf.setVerbose(true);

    std::printf("Configuring and starting module. \n");
    key_commands.runModule(rf);
    std::printf("Main returning..."); 

    return 0;
}
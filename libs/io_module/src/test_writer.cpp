#include <stdio.h>
#include <yarp/os/all.h>
#include <Eigen/Dense>
#include <iostream>

#include "reader.h"
#include "writer.h"

int main(int argc, char * argv[]) {
    yarp::os::Network yarp;


    int period = 10;
    ReadJoints rj(1000);
    WriteJoints wj(1000);
    // ReadCameras rc(100);

    rj.start();
    wj.start();
    //rc.start();
    yarp::os::Network::connect("/joints/read", "/joints/write");
    yarp::os::Time::delay(10);
    //rc.stop();
    wj.stop();
    rj.stop();

    return 0;
}
#include <stdio.h>
#include <yarp/os/all.h>
#include <Eigen/Dense>
#include <iostream>

#include "reader.h"

int main(int argc, char * argv[]) {
    yarp::os::Network yarp;


    int period = 10;
    // ReadJoints rj(1000);
    ReadCameras rc(100);

    // rj.start();
    rc.start();
    yarp::os::Time::delay(120);
    rc.stop();
    //rj.stop();
    //KeyReader();

    return 0;
}

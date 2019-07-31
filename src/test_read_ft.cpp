#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <iostream>

int main()
{
    yarp::os::Network yarp;

    // Read left and right ft sensors.
    yarp::os::BufferedPort<yarp::sig::Vector> lft;
    lft.open("/left_ft");

    yarp::os::Network::connect("/wholeBodyDynamics/left_leg/cartesianEndEffectorWrench:o", "/left_ft");

    yarp::sig::Vector* ftb = lft.read();

    for (int i = 0; i < (*ftb).size(); i++) {

        std::cout << (*ftb)[i] << std::endl;
    }


    yarp::os::Time::delay(10);

    return 0;
}
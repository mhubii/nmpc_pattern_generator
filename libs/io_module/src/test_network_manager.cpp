#include <stdio.h>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/FrameGrabberInterfaces.h>
#include <yarp/dev/Wrapper.h>
#include <yarp/dev/IRGBDSensor.h>
#include <yarp/dev/PolyDriver.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;

int main() {
    Network yarp;

    Property config;
    config.put("device", "remote_grabber");
    config.put("local", "/client");
    config.put("remote", "/icubGazeboSim/cam/left");

    PolyDriver dd(config);
    if (!dd.isValid()) {
        printf("Failed to create and configure device\n");
        return 1;
    }

    IFrameGrabberImage *grabberInterface;
    if(!dd.view(grabberInterface)) {
        printf("Failed to view device through IFrameGrabberImage interface\n");
        return 1;
    }

    ImageOf<PixelRgb> img;
    grabberInterface->getImage(img);
    printf("Got a %dx%d image\n", img.width(), img.height());

    file::write(img, "test_image");

    dd.close();

    return 0;
}
#include <stdio.h>

#include <opencv2/opencv.hpp>

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

    // Left image.
    Property config_left;
    config_left.put("device", "remote_grabber");
    config_left.put("local", "/client");
    config_left.put("remote", "/icubGazeboSim/cam/left");

    PolyDriver dd_left(config_left);
    if (!dd_left.isValid()) {
        printf("Failed to create and configure device\n");
        return 1;
    }

    IFrameGrabberImage *grabberInterface_left;
    if(!dd_left.view(grabberInterface_left)) {
        printf("Failed to view device through IFrameGrabberImage interface\n");
        return 1;
    }

    ImageOf<PixelRgb> img_left;
    grabberInterface_left->getImage(img_left);
    printf("Got a %dx%d image\n", img_left.width(), img_left.height());

    file::write(img_left, "test_image_left");

    dd_left.close();

    // Right image.    
    Property config_right;
    config_right.put("device", "remote_grabber");
    config_right.put("local", "/client");
    config_right.put("remote", "/icubGazeboSim/cam/right");

    PolyDriver dd_right(config_right);
    if (!dd_right.isValid()) {
        printf("Failed to create and configure device\n");
        return 1;
    }

    IFrameGrabberImage *grabberInterface_right;
    if(!dd_right.view(grabberInterface_right)) {
        printf("Failed to view device through IFrameGrabberImage interface\n");
        return 1;
    }

    ImageOf<PixelRgb> img_right;
    grabberInterface_right->getImage(img_right);
    printf("Got a %dx%d image\n", img_right.width(), img_right.height());

    file::write(img_right, "test_image_right");

    dd_right.close();

    // Create depth map. Convert to openCV format.
    cv::Mat cv_left = cv::cvarrToMat(img_left.getIplImage());
    cv::Mat cv_right = cv::cvarrToMat(img_right.getIplImage());

    cv::cvtColor(cv_left, cv_left, CV_BGR2GRAY);
    cv::cvtColor(cv_right, cv_right, CV_BGR2GRAY);

    // Determine disparity.
    cv::Mat disp;   

    cv::StereoBM sbm;
    sbm.state->SADWindowSize = 9;
    sbm.state->numberOfDisparities = 16;

    sbm(cv_left, cv_right, disp);
    cv::normalize(disp, disp, 0, 255, CV_MINMAX, CV_8U);

    cv::namedWindow("test", 1);
    cv::imshow("test", disp);

    // Free storage.
    cv::waitKey();

    return 0;
}
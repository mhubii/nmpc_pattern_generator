#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc.hpp>
#include <opencv2/highgui.hpp>
#include <yarp/os/all.h>

#include "reader.h"

int main() 
{
    yarp::os::Network yarp;

    int period_cam = 100;
    std::string io_config = "../../libs/io_module/configs.yaml";
    std::string robot = "icub";

    ReadCameras rc(period_cam, io_config, robot);


    // Ports to read images.
    std::map<std::string, yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb>>> ports_img;

    for (const auto& part : rc.GetParts()) {
        for (const auto& camera : part.cameras) {

            ports_img[camera].open("/tune_disp_map/" + camera);
            yarp::os::Network::connect("/read_cameras/" + camera, "/tune_disp_map/" + camera);
        }
    }

    rc.start();

    yarp::os::Time::delay(1);

    // Images of the cameras.
    std::map<std::string, yarp::sig::ImageOf<yarp::sig::PixelRgb>> imgs;
    std::map<std::string, cv::Mat> imgs_cv_rgb;

    for (int i = 1; i <= 60; i++) {

        // Read the camera images.
        for (const auto& part : rc.GetParts()) {
            for (const auto& camera : part.cameras) {

                yarp::sig::ImageOf<yarp::sig::PixelRgb>* img = ports_img[camera].read(false);
                if (img != YARP_NULLPTR) {

                    // Convert the images to a format that OpenCV uses.
                    imgs_cv_rgb[camera] = cv::cvarrToMat(img->getIplImage());
                    cv::cvtColor(imgs_cv_rgb[camera], imgs_cv_rgb[camera], cv::COLOR_BGR2RGB);

                    // Save images.
                    cv::imwrite("../../out/calib_imgs/" + camera + std::to_string(i) + ".jpg", imgs_cv_rgb[camera]);
                }
            }
        }

        yarp::os::Time::delay(1);
    }

    // Close everything.
    for (const auto& part : rc.GetParts()) {
        for (const auto& camera : part.cameras) {

            ports_img[camera].close();
        }
    }
    rc.stop();

    return 0;
}
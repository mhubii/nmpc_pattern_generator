#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <yarp/os/all.h>

#include "reader.h"

int main() 
{
    yarp::os::Network yarp;

    int period_cam = 100;
    std::string io_config = "../../libs/io_module/configs.yaml";
    std::string robot = "icubGazeboSim";

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
    std::map<std::string, cv::Mat> imgs_cv_gra;

    // Disparity map.
    cv::Mat l_disp; 
    cv::Mat r_disp; 
    cv::Mat wls_disp;

    // Stereo matching and weighted least square filter.
    cv::Ptr<cv::StereoBM> l_matcher = cv::StereoBM::create(32, 13);
    cv::Ptr<cv::StereoMatcher> r_matcher = cv::ximgproc::createRightMatcher(l_matcher);
    cv::Ptr<cv::ximgproc::DisparityWLSFilter> wls = cv::ximgproc::createDisparityWLSFilter(l_matcher);

    wls->setLambda(1e4);
    wls->setSigmaColor(1.);


    // Read the camera images.
    for (const auto& part : rc.GetParts()) {
        for (const auto& camera : part.cameras) {

            yarp::sig::ImageOf<yarp::sig::PixelRgb>* img = ports_img[camera].read(false);
            if (img != YARP_NULLPTR) {

                // Convert the images to a format that OpenCV uses.
                imgs_cv_rgb[camera] = cv::cvarrToMat(img->getIplImage());

                // Convert to gray image.
                cv::cvtColor(imgs_cv_rgb[camera], imgs_cv_gra[camera], cv::COLOR_BGR2GRAY);
                cv::cvtColor(imgs_cv_rgb[camera], imgs_cv_rgb[camera], cv::COLOR_BGR2RGB);
            }
        }
    }

    // Determine disparity.
    l_matcher->compute(imgs_cv_gra[rc.GetParts()[0].cameras[0]], imgs_cv_gra[rc.GetParts()[0].cameras[1]], l_disp);
    r_matcher->compute(imgs_cv_gra[rc.GetParts()[0].cameras[1]], imgs_cv_gra[rc.GetParts()[0].cameras[0]], r_disp);

    // Perform weighted least squares filtering.
    wls->filter(l_disp, imgs_cv_gra[rc.GetParts()[0].cameras[0]], wls_disp, r_disp);

    cv::ximgproc::getDisparityVis(l_disp, l_disp, 1);
    cv::ximgproc::getDisparityVis(r_disp, r_disp, -1);
    cv::ximgproc::getDisparityVis(wls_disp, wls_disp, 1);
    cv::normalize(l_disp, l_disp, 0, 255, CV_MINMAX, CV_8U);
    cv::normalize(r_disp, r_disp, 0, 255, CV_MINMAX, CV_8U);
    cv::normalize(wls_disp, wls_disp, 0, 255, CV_MINMAX, CV_8U);

    // Save images.
    std::string out_loc = "../../out/thesis/";
    cv::imwrite(out_loc + "rgb_left.jpg", imgs_cv_rgb["left"]); 
    cv::imwrite(out_loc + "rgb_right.jpg", imgs_cv_rgb["right"]); 
    cv::imwrite(out_loc + "gra_left.jpg", imgs_cv_gra["left"]);
    cv::imwrite(out_loc + "gra_right.jpg", imgs_cv_gra["right"]);
    cv::imwrite(out_loc + "left_disp.jpg", l_disp);
    cv::imwrite(out_loc + "right_disp.jpg", r_disp);
    cv::imwrite(out_loc + "wls_disp.jpg", wls_disp);

    for (const auto& part : rc.GetParts()) {
        for (const auto& camera : part.cameras) {

            ports_img[camera].close();
        }
    }
    rc.stop();

    return 0;
}

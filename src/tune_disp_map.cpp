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

    // Calibration.
    cv::Mat R1, R2, P1, P2, Q;
    cv::Mat K1, K2, R;
    cv::Vec3d T;
    cv::Mat D1, D2;

    std::string calib_file = "../../libs/io_module/cam_stereo.yml";
    cv::FileStorage fs1(calib_file, cv::FileStorage::READ);
    fs1["K1"] >> K1;
    fs1["K2"] >> K2;
    fs1["D1"] >> D1;
    fs1["D2"] >> D2;
    fs1["R"] >> R;
    fs1["T"] >> T;

    fs1["R1"] >> R1;
    fs1["R2"] >> R2;
    fs1["P1"] >> P1;
    fs1["P2"] >> P2;
    fs1["Q"] >> Q;

    cv::Mat lmapx, lmapy, rmapx, rmapy;

    // Read the camera images.
    for (const auto& part : rc.GetParts()) {
        for (const auto& camera : part.cameras) {

            yarp::sig::ImageOf<yarp::sig::PixelRgb>* img = ports_img[camera].read(false);
            if (img != YARP_NULLPTR) {

                // Convert the images to a format that OpenCV uses.
                imgs_cv_rgb[camera] = cv::cvarrToMat(img->getIplImage());
            }
        }
    }

    cv::initUndistortRectifyMap(K1, D1, R1, P1, imgs_cv_rgb["left"].size(), CV_32F, lmapx, lmapy);
    cv::initUndistortRectifyMap(K2, D2, R2, P2, imgs_cv_rgb["right"].size(), CV_32F, rmapx, rmapy);
      
    for (int i = 0; i < 1000; i++) {

        // Read the camera images.
        for (const auto& part : rc.GetParts()) {
            for (const auto& camera : part.cameras) {

                yarp::sig::ImageOf<yarp::sig::PixelRgb>* img = ports_img[camera].read(false);
                if (img != YARP_NULLPTR) {

                    // Convert the images to a format that OpenCV uses.
                    imgs_cv_rgb[camera] = cv::cvarrToMat(img->getIplImage());

                    if (camera == "left") {

                        cv::remap(imgs_cv_rgb[camera], imgs_cv_rgb[camera], lmapx, lmapy, cv::INTER_LINEAR);
                    }
                    else {

                        cv::remap(imgs_cv_rgb[camera], imgs_cv_rgb[camera], rmapx, rmapy, cv::INTER_LINEAR);
                    }

                    // Convert to gray image.
                    cv::cvtColor(imgs_cv_rgb[camera], imgs_cv_gra[camera], cv::COLOR_BGR2GRAY);
                }
            }
        }

        // Determine disparity.
        l_matcher->compute(imgs_cv_gra[rc.GetParts()[0].cameras[0]], imgs_cv_gra[rc.GetParts()[0].cameras[1]], l_disp);
        r_matcher->compute(imgs_cv_gra[rc.GetParts()[0].cameras[1]], imgs_cv_gra[rc.GetParts()[0].cameras[0]], r_disp);

        // Perform weighted least squares filtering.
        wls->filter(l_disp, imgs_cv_gra[rc.GetParts()[0].cameras[0]], wls_disp, r_disp);

        cv::ximgproc::getDisparityVis(wls_disp, wls_disp, 1);
        cv::normalize(wls_disp, wls_disp, 0, 255, CV_MINMAX, CV_8U);

        cv::imshow("WLS Disparity Map", wls_disp);
        cv::waitKey(10); // ms
    }

    for (const auto& part : rc.GetParts()) {
        for (const auto& camera : part.cameras) {

            ports_img[camera].close();
        }
    }
    rc.stop();

    return 0;
}

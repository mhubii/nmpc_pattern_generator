#ifndef IO_MODULE_READ_H_
#define IO_MODULE_READ_H_

#include <opencv2/opencv.hpp>
#include <opencv2/core/core_c.h>
#include <opencv2/ximgproc.hpp>

#include <stdio.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/FrameGrabberInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/eigen/Eigen.h>
#include <ncurses.h>
#include <yaml-cpp/yaml.h>

#include "utils.h"


// ReadJointsToFile implements a simple reader that
// reads joints from a YARP interface.
//
// Implemented by Martin Huber.
class ReadJoints : public yarp::os::RateThread
{
    public:
        
        ReadJoints(int period, const std::string config_file_loc = "../libs/io_module/configs.yaml", 
                   const std::string robot_name = "icubGazeboSim", const std::string out_file_loc = "../libs/io_module/data/states.csv");

        ~ReadJoints();

    private:

        // Methods to be implemented for RateThread.
        virtual void run();

        // Write data to file.
        //WriteCsv();

        // Configurations.
        void UnsetDrivers();

        void SetConfigs();

        void SetDrivers();

        // Robot.
        const std::string robot_name_;

        // Configurations.
        YAML::Node configs_;

        // Parts.
        std::vector<Part> parts_;

        // Output.
        std::string out_file_loc_;

        // Drivers.
        std::map<std::string, yarp::dev::PolyDriver*> dd_;
        std::map<std::string, yarp::dev::IEncoders*> enc_;
};


// ReadCameras implements a simple reader that
// reads camera images from a YARP interface.
//
// Implemented by Martin Huber.
class ReadCameras : public yarp::os::RateThread
{
    public:
        
        ReadCameras(int period, const std::string config_file_loc = "../libs/io_module/configs.yaml",
                   const std::string robot_name = "icubGazeboSim", const std::string out_file_loc = "../libs/io_module/data/img",
                   const std::string out_port_name = "/vel/prediction");

        ~ReadCameras();

    private:

        // Methods to be implemented for RateThread.
        virtual void run();

        // Configurations.
        void UnsetDrivers();

        void SetConfigs();

        void SetDrivers();

        // Robot.
        const std::string robot_name_;

        // run() is called every period_ ms.
        int period_;

        // Possible outputs.
        bool show_depth_view_;
        bool save_depth_view_;

        // Configurations.
        YAML::Node configs_;

        // Parts.
        std::vector<Part> parts_;

        // Output.
        std::string out_file_loc_;

        // Drivers.
        std::map<std::string, yarp::dev::PolyDriver*> dd_;
        std::map<std::string, yarp::dev::IFrameGrabberImage*> grab_;

        // Images of the cameras.
        std::map<std::string, yarp::sig::ImageOf<yarp::sig::PixelRgb>> img_;
        std::map<std::string, cv::Mat> img_cv_;

        // Stereo matching and weighted least square filter.
        cv::Ptr<cv::StereoBM> l_matcher_;
        cv::Ptr<cv::StereoMatcher> r_matcher_;
        cv::Ptr<cv::ximgproc::DisparityWLSFilter> wls_;

        // Disparity map.
        cv::Mat l_disp_; 
        cv::Mat r_disp_; 
        cv::Mat wls_disp_;

        // Outgoing information.
        yarp::sig::Vector vel_;

        // Outgoing port.
        yarp::os::BufferedPort<yarp::sig::Vector> port_;
};


// KeyReader implements a simple user interface that
// supports the user with w, a, s, d controls to write
// to a port via WriteToPort().
//
// Implemented by Martin Huber.
class KeyReader
{
    public:

        KeyReader();

        ~KeyReader();

    private:

        // Port for sending velocities.
        yarp::os::BufferedPort<yarp::sig::Vector> port_;

        // Read incomming commands and update the velocity.
        void ReadCommands();
    
        // Set velocity.
        void SetVelocity(Eigen::Vector3d& acc, double t);

        // Write to port.
        void WriteToPort();

        // Accelerations.
        Eigen::Vector3d acc_w_, acc_a_, acc_s_, acc_d_;

        // Respective time, accelerated in one direction.
        double t_iter_;

        // Velocity.
        yarp::sig::Vector vel_;

        // User interface.
        WINDOW *win_w_, *win_a_, *win_s_, *win_d_;
        WINDOW *win_q_, *win_e_, *win_info_, *win_vel_;
};

#endif
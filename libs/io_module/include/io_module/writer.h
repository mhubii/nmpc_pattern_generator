#ifndef IO_MODULE_WRITE_H_
#define IO_MODULE_WRITE_H_

#include <iostream>
#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yaml-cpp/yaml.h>
#include <yarp/eigen/Eigen.h>

#include "utils.h"

// Wrapper class for YARP to write to ports.
//
// Implemented by Martin Huber
class WriteJoints : public yarp::os::RateThread
{
    public:

        WriteJoints(int period, const std::string config_file_loc = "../libs/io_module/configs.yaml",
                    const std::string robot_name = "icubGazeboSim");

        ~WriteJoints();

        // Getters.
        inline const std::string&           GetPortName()              const { return port_name_; };
        inline const InitialPositionStatus& GetInitialPositionStatus() const { return moving_to_initial_pos_; };

    private:

        // Methods to be implemented for RateThread.
        virtual void run();

        // Implement methods for writing to YARP ports.
        void UnsetDrivers();
        
        void SetConfigs();

        void SetDrivers();

        bool SetControlModes(int mode);

        // Robot.
        const std::string robot_name_;

        // Configurations.
        YAML::Node configs_;

        // Parts.
        std::vector<Part> parts_;

        // Drivers.
        std::map<std::string, yarp::dev::PolyDriver*> dd_;
        std::map<std::string, yarp::dev::IControlMode2*> con_;
        std::map<std::string, yarp::dev::IPositionControl2*> pos_c_;
        std::map<std::string, yarp::dev::IPositionDirect*> pos_d_;

        // Moving to the initial position.
        InitialPositionStatus moving_to_initial_pos_;
        double initial_vel_;

        // Port to communicate initial position status.
        yarp::os::BufferedPort<yarp::os::Bottle> port_init_pos_status_;

        // Input.
        yarp::sig::Vector q_;

        // Port to read joint angles from.
        std::string port_name_;
        yarp::os::BufferedPort<yarp::sig::Vector> port_;
};

#endif
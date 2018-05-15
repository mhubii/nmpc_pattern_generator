#ifndef IO_MODULE_WRITE_H_
#define IO_MODULE_WRITE_H_

#include <iostream>
#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yaml-cpp/yaml.h>

#include "utils.h"

// Wrapper class for YARP to write to ports.
//
// Implemented by Martin Huber
<<<<<<< HEAD
class Writer
=======
class WriteJoints : public yarp::os::RateThread
>>>>>>> 5fd66e22f3cef2348c5b7fa0c12a6b6a07d04709
{
    public:

        WriteJoints(int period, const std::string config_file_loc = "../libs/io_module/configs.yaml",
                    const std::string robot_name = "icubGazeboSim");

        ~WriteJoints();

        // Getters.
        inline const std::string& GetPortName() const { return port_name_; };

    private:

        // Methods to be implemented for RateThread.
        virtual void run();

        // Implement methods for writing to YARP ports.
        void UnsetDrivers();
        
        void SetConfigs();

        void SetDrivers();

        // Robot.
        const std::string robot_name_;

        // Configurations.
        YAML::Node configs_;

        // Parts.
        std::vector<Part> parts_;

        // Drivers.
        std::map<std::string, yarp::dev::PolyDriver*> dd_;
        std::map<std::string, yarp::dev::IPositionControl*> pos_;

        // Input.
        yarp::sig::Vector q_;

        // Port to read joint angles from.
        std::string port_name_;
        yarp::os::BufferedPort<yarp::sig::Vector> port_;
};

#endif
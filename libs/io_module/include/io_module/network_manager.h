#ifndef IO_MODULE_NETWORK_MANAGER_H_
#define IO_MODULE_NETWORK_MANAGER_H_

#include <yarp/os/Network.h>
#include <yaml-cpp/yaml.h>
#include <vector>

#include "utils.h"

// Wrapper class for building up a
// YARP network.
//
// Implemented by Martin Huber.
class NetworkManager
{
    public:
        NetworkManager(const std::string config_file_loc);

    private:
        void OpenPorts();

        void SetConfigs(const std::string config_file_loc);

        // Configurations.
        YAML::Node configs_;

        std::vector<JointSensor> joints_;
};

#endif
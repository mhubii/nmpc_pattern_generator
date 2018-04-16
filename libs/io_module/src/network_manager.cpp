#include "network_manager.h"
#include <iostream>

NetworkManager::NetworkManager(const std::string config_file_loc) {
    // Network.
    yarp::os::Network yarp();

    // Set configurations.
    SetConfigs(config_file_loc);

    // Open ports for each sensor.
    OpenPorts();
}

void NetworkManager::OpenPorts() {
    
}

void NetworkManager::SetConfigs(const std::string config_file_loc) {

    // Load the configurations file.
    try {
        configs_ = YAML::LoadFile(config_file_loc);
    }
    catch (YAML::BadFile e) {
        std::cout << "Could not open file " << config_file_loc << std::endl;
        std::exit(1);
    }

    // Store the joints.
    for (YAML::const_iterator sensor = configs_["sensors"].begin();
         sensor != configs_["sensors"].end();
         sensor++) {

            for (YAML::const_iterator joint = (*sensor)["joints"].begin();
                 joint != (*sensor)["joints"].end();
                 joint++) {
                
                    joints_.push_back(JointSensor{(*sensor)["part"].as<std::string>(),
                                                  joint->as<std::size_t>()});
                }
        }
}

#include "writer.h"

WriteJoints::WriteJoints(int period, const std::string config_file_loc,
                         const std::string robot_name)
  : RateThread(period),
    robot_name_(robot_name),
    configs_(YAML::LoadFile(config_file_loc)) {

    // Set configurations and drivers.
    SetConfigs();
    SetDrivers();

    // Prepare the input.
    int joints = 0;

    for (auto& part : parts_) {
        joints += part.joints.size();
    }

    q_.resize(joints);

    // Open port to read from.
    port_.open(port_name_);
    
    if (port_.isClosed()) {
        std::cerr << "Could not open port " << port_name_ << std::endl;
    }
}


WriteJoints::~WriteJoints() {

    // Unset drivers.
    UnsetDrivers();

    // Close ports.
    port_.close();
}


void WriteJoints::run() {

    bool ok = true;
    int count = 0;

    // Run method implemented for RateThread, is called
    // every period ms. Here we want to write to the joints
    // of the robot defined in parts_.
    // Read data from a port.

    q_ = *port_.read(true);

    std::cout << q_[0] << std::endl;
}


void WriteJoints::UnsetDrivers() {

    // Close driver.
    for (const auto& part : parts_) {
        dd_[part.name]->close();
    }

    // Delete driver.
    for (const auto& part : parts_) {
        delete dd_[part.name];
    }
}


void WriteJoints::SetConfigs() {

    // Check for motors i.e. parts and their joints.
    for (YAML::const_iterator part = configs_["motors"].begin();
         part != configs_["motors"].end();
         part++) {
        
        if ((*part)["joints"]) {

            parts_.push_back(Part{(*part)["part"].as<std::string>(),
                                  (*part)["joints"].as<std::vector<int>>(),
                                  std::vector<std::string>()});
        }
    }

    // Check for the joints port name to write to.
    port_name_ = configs_["joints_port_write"].as<std::string>();

}


void WriteJoints::SetDrivers() {

    // Set a driver for each part.
    for (const auto& part : parts_) {

        std::string local_port = "/client_write/" + part.name;
        std::string remote_port = "/" + robot_name_ + "/" + part.name;

        yarp::os::Property options;
        options.put("device", "remote_controlboard");
        options.put("local", local_port);
        options.put("remote", remote_port);

        // Every part is set to position encoder.
        dd_[part.name] = new yarp::dev::PolyDriver(options);
        
        if (!dd_[part.name]->isValid()) 
        {
            std::cerr << "Device or ports not available." << std::endl;
            std::exit(1);
        }

        // Create IPositionControl interfaces for the motors.
        bool ok = true;

        yarp::dev::IPositionControl* p;
        ok = ok && dd_[part.name]->view(p);
        pos_[part.name] = p;

        if (!ok) 
        {
            std::cout << "Problems acquiring interfaces" << std::endl;
            std::exit(1);
        }
    }
}

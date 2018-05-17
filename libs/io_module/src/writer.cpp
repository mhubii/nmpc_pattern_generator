#include "writer.h"

WriteJoints::WriteJoints(int period, const std::string config_file_loc,
                         const std::string robot_name)
  : RateThread(period),
    robot_name_(robot_name),
    configs_(YAML::LoadFile(config_file_loc)),
    
    // Moving to the initial position.
    moving_to_initial_pos_(NOT_STARTED) {

    // Set configurations and drivers.
    SetConfigs();
    SetDrivers();

    // // Prepare the input.
    // int joints = 0;

    // for (auto& part : parts_) {
    //     joints += part.joints.size();
    // }

    // q_.resize(joints, );

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

    // Convert to degree.
    for (int i = 0; i < q_.rows(); i++) {

        for (int j = 0; j < q_.cols(); j++) {

            q_(i, j) *= RAD2DEG; 
        }
    }

    if (moving_to_initial_pos_ == NOT_STARTED) {
        
        // Set the control modes neccessary to reach the initial position.
        std::cout << "Moving to initial position." << std::endl;
        moving_to_initial_pos_ = MOVING;
        ok = ok && SetControlModes(VOCAB_CM_POSITION);

        for (auto& part : parts_) {

            // Set the reference speed for every joint and move to the initial position.
            ok = ok && pos_c_[part.name]->setRefSpeeds(part.joints.size(), &part.joints[0], &yarp::sig::Vector(part.joints.size(), initial_vel_)[0]);
            ok = ok && pos_c_[part.name]->positionMove(part.joints.size(), &part.joints[0], &q_.getCol(0)(count));

            count += part.joints.size();
        }
    }

    else if (moving_to_initial_pos_ == MOVING) {

        // Check if the initial position was reached for every joint of every part.
        bool done = true;

        for (auto& part : parts_) {

            for (auto& joint : part.joints) {
              
                bool motion_done;
                ok = ok && pos_c_[part.name]->checkMotionDone(joint, &motion_done);
                done = done && motion_done;
            }            
        }

        if (done) {

            // Change the control mode to IPositionDirect, once the initial position
            // was reached.
            std::cout << "Reached initial position." << std::endl;
            moving_to_initial_pos_ = DONE;
            ok = ok && SetControlModes(VOCAB_CM_POSITION_DIRECT);
        }
    }
    else {

        for (int i = 0; i < q_.cols(); i++) {
            
            for (auto& part : parts_) {

                // Set a reference position q_.
                ok = ok && pos_d_[part.name]->setPositions(part.joints.size(), &part.joints[0], &q_.getCol(i)(count));

                count += part.joints.size();
            }
        }
    }

    if (!ok) {

        std::cout << "Could not move motors." << std::endl;
        std::exit(1);
    }
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

    // Check the initial velocity.
    initial_vel_ = configs_["initial_vel"].as<double>()*RAD2DEG;

    // Check for the joints port name to read from.
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

        // Every part is set to position control and position direct.
        dd_[part.name] = new yarp::dev::PolyDriver(options);
        
        if (!dd_[part.name]->isValid()) 
        {
            std::cerr << "Device or ports not available." << std::endl;
            std::exit(1);
        }

        // Allow to change the control mode for every part. The robot
        // changes the control after it reached the initial position.
        bool ok = true;

        yarp::dev::IControlMode2* c;
        ok = ok && dd_[part.name]->view(c);
        con_[part.name] = c;

        yarp::dev::IPositionControl2* pc;
        ok = ok && dd_[part.name]->view(pc);
        pos_c_[part.name] = pc;

        yarp::dev::IPositionDirect* pd;
        ok = ok && dd_[part.name]->view(pd);
        pos_d_[part.name] = pd;

        if (!ok) 
        {
            std::cout << "Problems acquiring interfaces." << std::endl;
            std::exit(1);
        }
    }
}


bool WriteJoints::SetControlModes(int mode) {

    // Set the control modes for every joint of every part.
    bool ok = true;

    for (auto& part : parts_) {

        for (auto& i : part.joints) {

            ok = ok && con_[part.name]->setControlMode(i, mode);
        }
    }

    return ok;
}

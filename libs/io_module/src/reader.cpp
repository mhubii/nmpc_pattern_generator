#include "reader.h"

ReadJoints::ReadJoints(int period, const std::string config_file_loc, 
                       const std::string robot_name, const std::string out_file_loc)
  : RateThread(period),
    robot_name_(robot_name),
    configs_(YAML::LoadFile(config_file_loc)),
    out_file_loc_(out_file_loc) {

    // Set configurations and drivers.
    SetConfigs();
    SetDrivers();
    
    // Prepare the output.
    int joints = 0;

    for (auto& part : parts_) {
        joints += part.joints.size();
    }

    q_min_.resize(joints);
    q_max_.resize(joints);
    state_.resize(joints, 3);

    // Read extremal joint angles from the robot.
    ReadLimits();  

    // Open port to write to.
    port_.open(port_name_);
    
    if (port_.isClosed()) {
        std::cerr << "Could not open port " << port_name_ << std::endl;
    }
}


ReadJoints::~ReadJoints() {

    // Unset drivers.
    UnsetDrivers();

    // Close ports.
    port_.close();
}


void ReadJoints::run() {

    bool ok = true;
    int count = 0;

    // Run method implemented for RateThread, is called
    // every period ms. Here we want to read the joints
    // of the robot defined in parts_.
    for (const auto& part : parts_) {

        // Create an encoder for each joint.
        int ax = 0;
        ok = ok && enc_[part.name]->getAxes(&ax);
        yarp::sig::Vector pos(ax);
        yarp::sig::Vector vel(ax);
        yarp::sig::Vector acc(ax);
  
        // Read the encoders.
        ok = ok && enc_[part.name]->getEncoders(pos.data());
        ok = ok && enc_[part.name]->getEncoderSpeeds(vel.data());
        ok = ok && enc_[part.name]->getEncoderAccelerations(acc.data());

        // Store read encoders to state_.
        for (int i = 0; i < ax; i++) {
            state_(count, 0) = pos[i]*DEG2RAD;
            state_(count, 1) = vel[i]*DEG2RAD;
            state_(count, 2) = acc[i]*DEG2RAD;
            count++;
        }
    }

    if (!ok) {
        std::cout << "Could not read sensor data." << std::endl;
        std::exit(1);
    }

    // Send read data to a port.
    yarp::sig::Matrix& data = port_.prepare();
    data =   state_;
    port_.write();    
}


void ReadJoints::UnsetDrivers() {

    // Close driver.
    for (const auto& part : parts_) {
        dd_[part.name]->close();
    }

    // Delete driver.
    for (const auto& part : parts_) {
        delete dd_[part.name];
    }
}

void ReadJoints::SetConfigs() {
    
    // Check for sensors i.e. parts and their joints.
    for (YAML::const_iterator part = configs_["sensors"].begin();
         part != configs_["sensors"].end();
         part++) {
        
        if ((*part)["joints"]) {

            parts_.push_back(Part{(*part)["part"].as<std::string>(),
                                  (*part)["joints"].as<std::vector<int>>(),
                                  std::vector<std::string>()});
        }
    }

    // Check for the joints port name to write to.
    port_name_ = configs_["joints_port_read"].as<std::string>();
}


void ReadJoints::SetDrivers() {

    // Set a driver for each part.
    for (const auto& part : parts_) {

        std::string local_port = "/client/" + part.name;
        std::string remote_port = "/" + robot_name_ + "/" + part.name;

        yarp::os::Property options;
        options.put("device", "remote_controlboard");
        options.put("local", local_port);
        options.put("remote", remote_port);

        // Every part is set to position encoder.
        dd_[part.name] = new yarp::dev::PolyDriver(options);
        
        if (!dd_[part.name]->isValid()) {
            std::cerr << "Device or ports not available." << std::endl;
            std::exit(1);
        }

        // Create IEncoders interfaces for the sensors.
        bool ok = true;

        yarp::dev::IEncoders* e;
        ok = ok && dd_[part.name]->view(e);
        enc_[part.name] = e;

        yarp::dev::IControlLimits2* l;
        ok = ok && dd_[part.name]->view(l);
        lim_[part.name] = l;

        if (!ok) {
            std::cout << "Problems acquiring interfaces." << std::endl;
            std::exit(1);
        }
    }
}

void ReadJoints::ReadLimits() {

    bool ok = true;
    int count = 0;

    // Read the extremal angles of the joints.
    for (auto& part : parts_) {
        
        int ax = 0;

        ok = ok && enc_[part.name]->getAxes(&ax);

        for (int i = 0; i < ax; i++) {

            // Read limits.
            double min;
            double max;

            ok = ok && lim_[part.name]->getLimits(i, &min, &max);

            // Write the limits to an eigen matrix.
            q_min_(count) = min*DEG2RAD;
            q_max_(count) = max*DEG2RAD;
            
            count++;
        }
    }

    if (!ok) {
        std::cout << "Could not read sensor data." << std::endl;
        std::exit(1);
    }
}

ReadCameras::ReadCameras(int period, const std::string config_file_loc, 
                         const std::string robot_name, const std::string out_file_loc,
                         const std::string out_port_name)
  : RateThread(period),
    robot_name_(robot_name),
    period_(period),
    show_depth_view_(true),
    save_depth_view_(false),
    configs_(YAML::LoadFile(config_file_loc)),
    vel_(3) {
    
    // Stereo matching and weighted least square filter.
    l_matcher_ = cv::StereoBM::create(16, 9);
    r_matcher_ = cv::ximgproc::createRightMatcher(l_matcher_);
    wls_ = cv::ximgproc::createDisparityWLSFilter(l_matcher_);

    wls_->setLambda(1e3);
    wls_->setSigmaColor(1.5);

    // Outgoing velocity.
    vel_.zero();

    // Outgoing port.
    port_.open(out_port_name);

    // Set configurations and drivers.
    SetConfigs();
    SetDrivers();
}


ReadCameras::~ReadCameras() {

    // Unset drivers.
    UnsetDrivers();

    // Close ports.
    port_.close();
}


void ReadCameras::run() {

    // Read the camera every period_ ms.
    for (const auto& part : parts_) {
        for (const auto& camera : part.cameras) {
            grab_[camera]->getImage(img_[camera]);

            // Convert the images to a format that OpenCV uses.
            img_cv_[camera] = cv::cvarrToMat(img_[camera].getIplImage());

            // Convert to gray image.
            cv::cvtColor(img_cv_[camera], img_cv_[camera], cv::COLOR_BGR2GRAY);
        }
    }

    // Determine disparity.
    l_matcher_->compute(img_cv_[parts_[0].cameras[0]], img_cv_[parts_[0].cameras[1]], l_disp_);
    r_matcher_->compute(img_cv_[parts_[0].cameras[1]], img_cv_[parts_[0].cameras[0]], r_disp_);

    // Perform weighted least squares filtering.
    wls_->filter(l_disp_, img_cv_[parts_[0].cameras[0]], wls_disp_, r_disp_);

    //cv::ximgproc::getDisparityVis(wls_disp_, wls_disp_, 1);
    cv::normalize(wls_disp_, wls_disp_, 0, 255, CV_MINMAX, CV_8U);

    // Show and or save the depth view.
    if (show_depth_view_) {
        cv::namedWindow("Depth View", cv::WINDOW_AUTOSIZE);
        cv::imshow("Depth View", wls_disp_);
        cv::waitKey(period_);
    }

    if (save_depth_view_) {

    }

    // Apply neural net to the depth view to make a decision.
    yarp::sig::Vector& data = port_.prepare();
    data = vel_;
    port_.write();       
}


void ReadCameras::UnsetDrivers() {
    
    // Close driver.
    for (const auto& part : parts_) {
        for (const auto& camera : part.cameras) {
            dd_[camera]->close();
        }
    }

    // Delete driver.
    for (const auto& part : parts_) {
        for (const auto& camera : part.cameras) {
            delete dd_[camera];
        }
    }
}


void ReadCameras::SetConfigs() {
    
    // Check for sensors i.e. parts and their cameras.
    for (YAML::const_iterator part = configs_["sensors"].begin();
            part != configs_["sensors"].end();
            part++) {
        
        if ((*part)["cameras"]) {
            parts_.push_back(Part{(*part)["part"].as<std::string>(),
                                  std::vector<int>(),
                                  (*part)["cameras"].as<std::vector<std::string>>()});
        }
    }
}


void ReadCameras::SetDrivers() {

    // Set a driver for each camera.
    for (const auto& part : parts_) {
        for (const auto& camera : part.cameras) {

            std::string local_port = "/client/cam/" + camera;
            std::string remote_port = "/" + robot_name_ + "/cam/" + camera;

            yarp::os::Property options;
            options.put("device", "remote_grabber");
            options.put("local", local_port);
            options.put("remote", remote_port);

            dd_[camera] = new yarp::dev::PolyDriver(options);

            if (!dd_[camera]->isValid()) 
            {
                std::cerr << "Device or ports not available." << std::endl;
                std::exit(1);
            }

            //
            bool ok = true;

            yarp::dev::IFrameGrabberImage* f;
            ok = ok && dd_[camera]->view(f);
            grab_[camera] = f;

            if (!ok) 
            {
                std::cout << "Problems acquiring interfaces" << std::endl;
                std::exit(1);
            }
        }
    }
}


AppReader::AppReader() 
    : running_(false),
      robot_status_(NOT_CONNECTED),
      errors_(NO_ERRORS),
      warnings_(NO_WARNINGS),
      vel_(3) {

    // Open ports.
    port_vel_app_.open("/vel/read_from_app");
    port_vel_.open("/vel/command");
    port_status_.open("/status/command");

    // Set velocity to zero.
    vel_.zero();

    // User interface.
    initscr();
    timeout(10);
    noecho();
    cbreak();
    curs_set(0);

    win_q_ = newwin(3, 6,  2, 2);
    win_e_ = newwin(3, 6,  2, 10);
    win_r_ = newwin(3, 6,  2, 18);
    win_guide_ = newwin(20, 44, 2, 28);
    win_robot_status_ = newwin(2, 22, 10, 2);
    win_err_ = newwin(2, 22, 13, 2);
    win_vel_ = newwin(6, 20, 17, 2);
    win_inv_ = newwin(1, 1, 22, 2);

    start_color();
    init_pair(1, COLOR_WHITE, COLOR_BLUE);
    init_pair(2, COLOR_WHITE, COLOR_YELLOW);
    init_pair(3, COLOR_WHITE, COLOR_RED);
    init_pair(4, COLOR_WHITE, COLOR_GREEN);
    init_pair(5, COLOR_BLUE, COLOR_WHITE);

    bkgd(COLOR_PAIR(1));
    wbkgd(win_q_, COLOR_PAIR(2));
    wbkgd(win_e_, COLOR_PAIR(5));
    wbkgd(win_r_, COLOR_PAIR(5));
    wbkgd(win_guide_, COLOR_PAIR(1));
    wbkgd(win_robot_status_, COLOR_PAIR(3));
    wbkgd(win_err_, COLOR_PAIR(4));
    wbkgd(win_vel_, COLOR_PAIR(1));
    wbkgd(win_inv_, COLOR_PAIR(1));

    mvwaddstr(win_q_, 1, 2, "q");
    mvwaddstr(win_e_, 1, 2, "e");
    mvwaddstr(win_r_, 1, 2, "r");
    mvwaddstr(win_guide_, 0, 0, ("Hello, I am the app user interface of the heicub robot.\n\n"
                                 "Create a hotspot on your smartphone and connect to it, exit with q if you have not already done this.\n\n"
                                 "Start your app on the smartphone and open the menu bar, choose settings.\n\n"
                                 "Type in the following settings:\n\n"
                                 "    IP Address:   " + port_vel_app_.where().getHost() + "\n"
                                 "    Port Send:    " + std::to_string(port_vel_app_.where().getPort()) + "\n"
                                 "    Port Receive: " + std::to_string(port_status_.where().getPort()) + "\n\n"
                                 "Go to the menu and choose connection. Press connect. Choose your favorite Joystick.\n"
                                 "Press r to run the robot. For an emergency stop press e.\n\n").c_str()); 

    mvwaddstr(win_robot_status_, 0, 2, "Robot:\n"
                                       "  Not connected!");
    mvwaddstr(win_err_, 0, 2, "Warnings:\n"
                              "  No warnings :)");

    mvwaddstr(win_vel_, 0, 0, ("Current velocity:\n\n"
                               "v_x:   " + std::to_string(vel_(0)) + "\n"
                               "v_y:   " + std::to_string(vel_(1)) + "\n"
                               "v_ang: " + std::to_string(vel_(2))).c_str());

    refresh();
    wrefresh(win_q_);
    wrefresh(win_e_);
    wrefresh(win_r_);
    wrefresh(win_guide_);
    wrefresh(win_robot_status_);
    wrefresh(win_err_);
    wrefresh(win_vel_);
    wrefresh(win_inv_);

    // Callback and callbacklock.
    useCallback();
    setCallbackLock(&mutex_);
}


AppReader::~AppReader() {

    // Set velocity to zero before closing.
    vel_.zero();
    WriteToPort();

    // Close port.
    port_vel_app_.close();
    port_vel_.close();
    port_status_.close();

    // Release the user interface.
    delwin(win_q_);
    delwin(win_e_);
    delwin(win_r_);
    delwin(win_guide_);
    delwin(win_robot_status_);
    delwin(win_err_);
    delwin(win_vel_);
    delwin(win_inv_);

    clrtoeol();
    refresh();
    endwin();
}


void AppReader::onRead(yarp::os::Bottle& info) {

    lockCallback();

    yarp::os::Value prop = info.pop();

    // Check for errors and warnings.
    if (prop.asDict()->check("ERROR")) {

        switch(prop.asDict()->find("ERROR").asInt()) {

            case NO_ERRORS: {

                // Communicate with the app.
                yarp::os::Bottle& bottle = port_status_.prepare();
                yarp::os::Property& dict = bottle.addDict();

                dict.put("ERROR", NO_ERRORS);
                port_status_.write();

                // Communicate with the terminal.
                errors_ = NO_ERRORS;
                wclear(win_err_);
                wbkgd(win_err_, COLOR_PAIR(4));
                mvwaddstr(win_err_, 0, 2, "Warnings:\n"
                                          "  No warnings :)");
                wrefresh(win_err_);
                break;
            }

            case QP_INFEASIBLE: {

                // Communicate with the app.
                yarp::os::Bottle& bottle = port_status_.prepare();
                yarp::os::Property& dict = bottle.addDict();

                dict.put("ERROR", QP_INFEASIBLE);
                port_status_.write();

                // Communicate with the terminal.
                errors_ = QP_INFEASIBLE;
                wclear(win_err_);
                wbkgd(win_err_, COLOR_PAIR(3));
                mvwaddstr(win_err_, 0, 2, "Warnings:\n"
                                          "  QP infeasible!");
                wrefresh(win_err_);

                // Reaction to infeasible qp.
                robot_status_ = NOT_CONNECTED;
                wclear(win_robot_status_);
                wbkgd(win_robot_status_, COLOR_PAIR(4));
                mvwaddstr(win_robot_status_, 0, 2, "Robot:\n"
                                                   "  Removed connection.");
                wrefresh(win_robot_status_);
                break;
            }

            case HARDWARE_LIMITS: {

                // Communicate with the app.
                yarp::os::Bottle& bottle = port_status_.prepare();
                yarp::os::Property& dict = bottle.addDict();

                dict.put("ERROR", HARDWARE_LIMITS);
                port_status_.write();

                // Communicate with the terminal.
                errors_ = HARDWARE_LIMITS;
                wclear(win_err_);
                wbkgd(win_err_, COLOR_PAIR(3));
                mvwaddstr(win_err_, 0, 2, "Warnings:\n"
                                          "  Hardware limits!");
                wrefresh(win_err_);

                // Reaction to hardware limits.
                robot_status_ = NOT_CONNECTED;
                wclear(win_robot_status_);
                wbkgd(win_robot_status_, COLOR_PAIR(4));
                mvwaddstr(win_robot_status_, 0, 2, "Robot:\n"
                                                   "  Removed connection.");
                wrefresh(win_robot_status_);
                break;
            }
        }
    }

    if (prop.asDict()->check("Warning")) {

        switch(prop.asDict()->find("Warning").asInt()) {

            case NO_WARNINGS: {

                // Communicate with the app.
                yarp::os::Bottle& bottle = port_status_.prepare();
                yarp::os::Property& dict = bottle.addDict();

                dict.put("Warning", NO_WARNINGS);
                port_status_.write();

                // Communicate with the terminal.
                warnings_ = NO_WARNINGS;
                wclear(win_err_);
                wbkgd(win_err_, COLOR_PAIR(4));
                mvwaddstr(win_err_, 0, 2, "Warnings:\n"
                                          "  No warnings :)");
                wrefresh(win_err_);
                break;
            }

            case IK_DID_NOT_CONVERGE: {

                // Communicate with the app.
                yarp::os::Bottle& bottle = port_status_.prepare();
                yarp::os::Property& dict = bottle.addDict();

                dict.put("Warning", IK_DID_NOT_CONVERGE);
                port_status_.write();

                // Communicate with the terminal.
                warnings_ = NO_WARNINGS;
                wclear(win_err_);
                wbkgd(win_err_, COLOR_PAIR(2));
                mvwaddstr(win_err_, 0, 2, "Warnings:\n"
                                          "  IK did not converge.");
                wrefresh(win_err_);
                break;
            }
        }
    }

    // Check for status of the robot.
    if (prop.asDict()->check("RobotStatus")) {
        
        switch (prop.asDict()->find("RobotStatus").asInt()) {

            case NOT_CONNECTED: {

                // Communicate with the app.
                yarp::os::Bottle& bottle = port_status_.prepare();
                yarp::os::Property& dict = bottle.addDict();

                dict.put("RobotStatus", NOT_CONNECTED);
                port_status_.write();

                // Communicate with the terminal.
                robot_status_ = NOT_CONNECTED;
                wclear(win_robot_status_);
                wbkgd(win_robot_status_, COLOR_PAIR(3));
                mvwaddstr(win_robot_status_, 0, 2, "Robot:\n"
                                                   "  Not connected!");
                wrefresh(win_robot_status_);
                break;
            }

            case NOT_INITIALIZED: {

                // Communicate with the app.
                yarp::os::Bottle& bottle = port_status_.prepare();
                yarp::os::Property& dict = bottle.addDict();

                dict.put("RobotStatus", NOT_INITIALIZED);
                port_status_.write();

                // Communicate with the terminal.
                robot_status_ = NOT_INITIALIZED;
                wclear(win_robot_status_);
                wbkgd(win_robot_status_, COLOR_PAIR(2));
                mvwaddstr(win_robot_status_, 0, 2, "Robot:\n"
                                                   "  Not initialized!");
                wrefresh(win_robot_status_);
                break;
            }
        
            case INITIALIZING: {

                // Communicate with the app.
                yarp::os::Bottle& bottle = port_status_.prepare();
                yarp::os::Property& dict = bottle.addDict();

                dict.put("RobotStatus", INITIALIZING);
                port_status_.write();

                // Communicate with the terminal.
                robot_status_ = INITIALIZING;
                wclear(win_robot_status_);
                wbkgd(win_robot_status_, COLOR_PAIR(2));
                mvwaddstr(win_robot_status_, 0, 2, "Robot:\n"
                                                   "  Initializing...");
                wrefresh(win_robot_status_);
                break;
            }

            case INITIALIZED: {

                // Communicate with the app.
                yarp::os::Bottle& bottle = port_status_.prepare();
                yarp::os::Property& dict = bottle.addDict();

                dict.put("RobotStatus", INITIALIZED);
                port_status_.write();

                // Communicate with the terminal.
                robot_status_ = INITIALIZED;
                wclear(win_robot_status_);
                wbkgd(win_robot_status_, COLOR_PAIR(4));
                mvwaddstr(win_robot_status_, 0, 2, "Robot:\n"
                                                   "  Good to go!");
                wrefresh(win_robot_status_);
                break;
            }
        }
    }

    unlockCallback();
}

void AppReader::ReadCommands() {
    
    // Read the pressed keys to quit or for
    // an emergency stop. Set the velocities
    // as read from the app and write them to a port.
    int ch = 0;

    do {                
        
        // Read input periodically.
        ch = getch();

        if (!running_ && ch == 'r') {

            running_ = true;

            // Run user controlled walking.
            std::system("gnome-terminal -x bash /home/martin/Documents/rl_gazebo_presentation/macros/run_user_controlled_walking.sh");
        }

        if (robot_status_ == INITIALIZED) {
            
        yarp::os::Bottle* vel = port_vel_app_.read(false);

            if (vel != YARP_NULLPTR) {

                vel->get(0).asList()->write(vel_);

                // Weight inputs.
                vel_(0) *=  0.6;
                vel_(1) *= -0.6;
                vel_(2) *=  0.1;

                WriteToPort();
            }
        }
        
        // Update user interface.  
        mvwaddstr(win_vel_, 0, 0, ("Current velocity:\n\n"
                                    "v_x:   " + std::to_string(vel_(0)) + "\n"
                                    "v_y:   " + std::to_string(vel_(1)) + "\n"
                                    "v_ang: " + std::to_string(vel_(2))).c_str());
        wrefresh(win_vel_);
        //wclear(win_inv_);
        //wrefresh(win_inv_);

    } while (ch != 'q' && ch != 'Q'); // Exit if q or Q is pressed.
}

void AppReader::WriteToPort() {

    // Write the velocities to the output port.
    yarp::sig::Vector& data = port_vel_.prepare();
    data = vel_;
    port_vel_.write();
}


KeyReader::KeyReader() 
    : robot_status_(NOT_CONNECTED),
      errors_(NO_ERRORS),
      warnings_(NO_WARNINGS),
      t_iter_(yarp::os::Time::now()), 
      acc_w_( 1., 0., 0. ),
      acc_a_( 0. , 0., -1.),
      acc_shift_a_(0., 1., 0.),
      acc_s_(-1., 0., 0.),
      acc_d_( 0. , 0., 1.),
      acc_shift_d_(0., -1., 0.),
      vel_(3) {

    // Open port.
    port_.open("/vel/command");

    // Set velocity to zero.
    vel_.zero();

    // User interface.
    initscr();
    timeout(10);
    noecho();
    cbreak();
    curs_set(0);

    win_w_ = newwin( 3, 6,  2, 10);
    win_a_ = newwin( 3, 6,  6,  2);
    win_s_ = newwin( 3, 6,  6, 10);
    win_d_ = newwin( 3, 6,  6, 18);
    win_q_ = newwin( 3, 6,  2,  2);
    win_e_ = newwin( 3, 6,  2, 18);
    win_guide_ = newwin(20, 44, 2, 28);
    win_robot_status_ = newwin(2, 22, 10, 2);
    win_err_ = newwin(2, 22, 13, 2);
    win_vel_ = newwin(6, 20, 16, 2);
    win_inv_ = newwin(1, 1, 22, 2);

    start_color();
    init_pair(1, COLOR_WHITE, COLOR_BLUE);
    init_pair(2, COLOR_BLUE, COLOR_WHITE);
    init_pair(3, COLOR_WHITE, COLOR_YELLOW);
    init_pair(4, COLOR_WHITE, COLOR_RED);
    init_pair(5, COLOR_WHITE, COLOR_GREEN);

    bkgd(COLOR_PAIR(1));
    wbkgd(win_w_, COLOR_PAIR(2));
    wbkgd(win_a_, COLOR_PAIR(2));
    wbkgd(win_s_, COLOR_PAIR(2));
    wbkgd(win_d_, COLOR_PAIR(2));
    wbkgd(win_q_, COLOR_PAIR(3));
    wbkgd(win_e_, COLOR_PAIR(3));
    wbkgd(win_guide_, COLOR_PAIR(1));
    wbkgd(win_robot_status_, COLOR_PAIR(4));
    wbkgd(win_err_, COLOR_PAIR(5));
    wbkgd(win_vel_, COLOR_PAIR(1));
    wbkgd(win_inv_, COLOR_PAIR(1));

    mvwaddstr(win_w_, 1, 2, "w");
    mvwaddstr(win_a_, 1, 2, "a");
    mvwaddstr(win_s_, 1, 2, "s");
    mvwaddstr(win_d_, 1, 2, "d");
    mvwaddstr(win_q_, 1, 2, "q");
    mvwaddstr(win_e_, 1, 2, "e");
    mvwaddstr(win_guide_, 0, 0, "Hello, I am the keyboard user interface of the heicub robot.\n\n"
                                "Use w and s to accelerate forwards and backwards.\n\n"
                                "Use a and d to accelerate angular left and right.\n\n"
                                "Use shift+a and shift+d to accelerate left and right.\n\n"
                                "Only one pressed key is used to accelerate, leaving all other velocities untouched, except for the opposite ones.\n\n"
                                "For an emergency stop press e.\n\n"
                                "Press q to quit this interface."); 
    mvwaddstr(win_robot_status_, 0, 2, "Robot:\n"
                                        "  Not connected!");
    mvwaddstr(win_err_, 0, 2, "Warnings:\n"
                            "  No warnings :)");
    mvwaddstr(win_vel_, 0, 0, ("Current velocity:\n\n"
                                "v_x:   " + std::to_string(vel_(0)) + "\n"
                                "v_y:   " + std::to_string(vel_(1)) + "\n"
                                "v_ang: " + std::to_string(vel_(2))).c_str());

    refresh();
    wrefresh(win_w_);
    wrefresh(win_a_);
    wrefresh(win_s_);
    wrefresh(win_d_);
    wrefresh(win_q_);
    wrefresh(win_e_);
    wrefresh(win_guide_);
    wrefresh(win_robot_status_);
    wrefresh(win_err_);
    wrefresh(win_vel_);
    wrefresh(win_inv_);

    // Callback and callbacklock.
    useCallback();
    setCallbackLock(&mutex_);
}

KeyReader::~KeyReader() {

    // Set velocity to zero before closing.
    vel_.zero();
    WriteToPort();

    // Close port.
    port_.close();

    // Release the user interface.
    delwin(win_w_);
    delwin(win_a_);
    delwin(win_s_);
    delwin(win_d_);
    delwin(win_q_);
    delwin(win_e_);
    delwin(win_guide_);
    delwin(win_robot_status_);
    delwin(win_err_);
    delwin(win_vel_);
    delwin(win_inv_);

    clrtoeol();
    refresh();
    endwin();
}

void KeyReader::onRead(yarp::os::Bottle& info) {

    lockCallback();

    yarp::os::Value prop = info.pop();

    // Check for errors and warnings.
    if (prop.asDict()->check("ERROR")) {

        switch(prop.asDict()->find("ERROR").asInt()) {

            case NO_ERRORS:
                errors_ = NO_ERRORS;
                wclear(win_err_);
                wbkgd(win_err_, COLOR_PAIR(5));
                mvwaddstr(win_err_, 0, 2, "Warnings:\n"
                                          "  No warnings :)");
                wrefresh(win_err_);
                break;

            case QP_INFEASIBLE:
                errors_ = QP_INFEASIBLE;
                wclear(win_err_);
                wbkgd(win_err_, COLOR_PAIR(4));
                mvwaddstr(win_err_, 0, 2, "Warnings:\n"
                                          "  QP infeasible!");
                wrefresh(win_err_);

                // Reaction to infeasible qp.
                robot_status_ = NOT_CONNECTED;
                wclear(win_robot_status_);
                wbkgd(win_robot_status_, COLOR_PAIR(5));
                mvwaddstr(win_robot_status_, 0, 2, "Robot:\n"
                                                   "  Removed connection.");
                wrefresh(win_robot_status_);
                break;

            case HARDWARE_LIMITS:
                errors_ = HARDWARE_LIMITS;
                wclear(win_err_);
                wbkgd(win_err_, COLOR_PAIR(4));
                mvwaddstr(win_err_, 0, 2, "Warnings:\n"
                                          "  Hardware limits!");
                wrefresh(win_err_);

                // Reaction to hardware limits.
                robot_status_ = NOT_CONNECTED;
                wclear(win_robot_status_);
                wbkgd(win_robot_status_, COLOR_PAIR(5));
                mvwaddstr(win_robot_status_, 0, 2, "Robot:\n"
                                                   "  Removed connection.");
                wrefresh(win_robot_status_);
                break;
        }
    }

    if (prop.asDict()->check("Warning")) {

        switch(prop.asDict()->find("Warning").asInt()) {

            case NO_WARNINGS:
                warnings_ = NO_WARNINGS;
                wclear(win_err_);
                wbkgd(win_err_, COLOR_PAIR(5));
                mvwaddstr(win_err_, 0, 2, "Warnings:\n"
                                          "  No warnings :)");
                wrefresh(win_err_);
                break;

            case IK_DID_NOT_CONVERGE:
                warnings_ = NO_WARNINGS;
                wclear(win_err_);
                wbkgd(win_err_, COLOR_PAIR(3));
                mvwaddstr(win_err_, 0, 2, "Warnings:\n"
                                          "  IK did not converge.");
                wrefresh(win_err_);
                break;

        }
    }

    // Check for status of the robot.
    if (prop.asDict()->check("RobotStatus")) {
        
        switch (prop.asDict()->find("RobotStatus").asInt()) {

            case NOT_CONNECTED:
                robot_status_ = NOT_CONNECTED;
                wclear(win_robot_status_);
                wbkgd(win_robot_status_, COLOR_PAIR(4));
                mvwaddstr(win_robot_status_, 0, 2, "Robot:\n"
                                                   "  Not connected!");
                wrefresh(win_robot_status_);
                break;

            case NOT_INITIALIZED:
                robot_status_ = NOT_INITIALIZED;
                wclear(win_robot_status_);
                wbkgd(win_robot_status_, COLOR_PAIR(4));
                mvwaddstr(win_robot_status_, 0, 2, "Robot:\n"
                                                   "  Not initialized!");
                wrefresh(win_robot_status_);
                break;
        
            case INITIALIZING:
                robot_status_ = INITIALIZING;
                wclear(win_robot_status_);
                wbkgd(win_robot_status_, COLOR_PAIR(3));
                mvwaddstr(win_robot_status_, 0, 2, "Robot:\n"
                                                   "  Initializing...");
                wrefresh(win_robot_status_);
                break;

            case INITIALIZED:
                robot_status_ = INITIALIZED;
                wclear(win_robot_status_);
                wbkgd(win_robot_status_, COLOR_PAIR(5));
                mvwaddstr(win_robot_status_, 0, 2, "Robot:\n"
                                                   "  Good to go!");
                wrefresh(win_robot_status_);
                break;
        }
    }

    unlockCallback();
};

void KeyReader::ReadCommands() {

    // Read the pressed keys, set the velocities
    // and write them to the port.
    int ch = 0;
    double dt = 0.;
    double t = 0.;

    do {                
        // Read input periodically.
        ch = getch();
     
        // Measure time between successive inputs.
        t_iter_ = yarp::os::Time::now() - t_iter_;
        dt = t_iter_;
        t_iter_ = yarp::os::Time::now();

        if (robot_status_ == INITIALIZED) {
            switch(ch)
            {
                case 'w':
                    SetVelocity(acc_w_, dt);
                    WriteToPort();
                    break;
                case 'a':
                    SetVelocity(acc_a_, dt);
                    WriteToPort();
                    break;
                case 'A':
                    SetVelocity(acc_shift_a_, dt);
                    WriteToPort();
                    break;
                case 's':
                    SetVelocity(acc_s_, dt);
                    WriteToPort();
                    break;
                case 'd':
                    SetVelocity(acc_d_, dt);
                    WriteToPort();
                    break;
                case 'D':
                    SetVelocity(acc_shift_d_, dt);
                    WriteToPort();
                    break;
                case 'e':
                    vel_.zero();
                    WriteToPort();
                    break;
            }
        }

        // Update user interface.  
        mvwaddstr(win_vel_, 0, 0, ("Current velocity:\n\n"
                                   "v_x:   " + std::to_string(vel_(0)) + "\n"
                                   "v_y:   " + std::to_string(vel_(1)) + "\n"
                                   "v_ang: " + std::to_string(vel_(2))).c_str());
        wrefresh(win_vel_);
        wclear(win_inv_);
        wrefresh(win_inv_);

    } while (ch != 'q' && ch != 'Q'); // Exit if q or Q is pressed.
}

void KeyReader::SetVelocity(Eigen::Vector3d& acc, double t) {

    // Determine the new velocity, given the previous one,
    // and the acceleration.
    Eigen::Map<Eigen::VectorXd> vel = yarp::eigen::toEigen(vel_);
    vel += acc*t;
}

void KeyReader::WriteToPort() {
    
    // Write the velocities to the output port.
    yarp::sig::Vector& data = port_.prepare();
    data = vel_;
    port_.write();
}

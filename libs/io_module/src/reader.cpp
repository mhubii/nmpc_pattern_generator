#include "reader.h"

#include <iostream>


ReadJoints::ReadJoints(int period, const std::string config_file_loc, 
                       const std::string robot_name, const std::string out_file_loc)
  : RateThread(period),
    robot_name_(robot_name),
    configs_(YAML::LoadFile(config_file_loc)),
    out_file_loc_(out_file_loc) {

    // Set configurations and drivers.
    SetConfigs();
    SetDrivers();
}

ReadJoints::~ReadJoints() {

    // Unset drivers.
    UnsetDrivers();
}


void ReadJoints::run() {

    // Run method implemented for RateThread, is called
    // every period ms. Here we want to read the joints
    // of the robot defined in parts_.
    for (const auto& part : parts_) {
        

        // Then, the read data is sent to the ports
        // opened in SetDrivers().
    }


    
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
        
        if (!dd_[part.name]->isValid()) 
        {
            std::cerr << "Device or ports not available." << std::endl;
            std::exit(1);
        }

        // Create IEncoders interfaces for the sensors.
        bool ok = true;

        yarp::dev::IEncoders* e;
        ok = ok && dd_[part.name]->view(e);
        enc_[part.name] = e;

        if (!ok) 
        {
            std::cout << "Problems acquiring interfaces" << std::endl;
            std::exit(1);
        }
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





KeyReader::KeyReader() 
    : t_iter_(yarp::os::Time::now()), 
      acc_w_( 0.1, 0.,  0. ),
      acc_a_( 0. , 0.,  0.1),
      acc_s_(-0.1, 0.,  0. ),
      acc_d_( 0. , 0., -0.1),
      vel_(3) {

  // Open port.
  port_.open("/vel/command");

  // Set accelerations and velocity.
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
  win_info_ = newwin(18, 44, 2, 28);
  win_vel_ = newwin(6, 20, 15, 2);

  start_color();
  init_pair(1, COLOR_WHITE, COLOR_BLUE);
  init_pair(2, COLOR_BLUE, COLOR_WHITE);
  init_pair(3, COLOR_WHITE, COLOR_YELLOW);

  bkgd(COLOR_PAIR(1));
  wbkgd(win_w_, COLOR_PAIR(2));
  wbkgd(win_a_, COLOR_PAIR(2));
  wbkgd(win_s_, COLOR_PAIR(2));
  wbkgd(win_d_, COLOR_PAIR(2));
  wbkgd(win_q_, COLOR_PAIR(3));
  wbkgd(win_e_, COLOR_PAIR(3));
  wbkgd(win_info_, COLOR_PAIR(1));
  wbkgd(win_vel_, COLOR_PAIR(1));

  mvwaddstr(win_w_, 1, 2, "w");
  mvwaddstr(win_a_, 1, 2, "a");
  mvwaddstr(win_s_, 1, 2, "s");
  mvwaddstr(win_d_, 1, 2, "d");
  mvwaddstr(win_q_, 1, 2, "q");
  mvwaddstr(win_e_, 1, 2, "e");
  mvwaddstr(win_info_, 0, 0, "Hello,\n\n"
                          "I am the keyboard user interface of the heicub robot.\n\n"
                          "Use w and s to accelerate forwards and backwards.\n\n"
                          "Use a and d to accelerate angular left and right.\n\n"
                          "Only one pressed key is used to accelerate, leaving all other velocities untouched, except for the opposite ones.\n\n"
                          "For an emergency stop press e.\n\n"
                          "Press q to quit this interface."); 
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
  wrefresh(win_info_);
  wrefresh(win_vel_);

  // Read incomming commands.
  ReadCommands();
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
  delwin(win_info_);
  delwin(win_vel_);

  clrtoeol();
  refresh();
  endwin();
}

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

        switch(ch)
        {
            case 'w': {
                SetVelocity(acc_w_, dt);
                WriteToPort();
                break;}
            case 'a':
                SetVelocity(acc_a_, dt);
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
            case 'e':
                vel_.zero();
                WriteToPort();
                break;
        }

        // Update user interface.  
        mvwaddstr(win_vel_, 0, 0, ("Current velocity:\n\n"
                                   "v_x:   " + std::to_string(vel_(0)) + "\n"
                                   "v_y:   " + std::to_string(vel_(1)) + "\n"
                                   "v_ang: " + std::to_string(vel_(2))).c_str());
        wrefresh(win_vel_);

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

#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc.hpp>
#include <chrono>
#include <yarp/os/all.h>
#include <Eigen/Core>
#include <rbdl/rbdl.h>
#include <qpOASES.hpp>

#include "reader.h"
#include "writer.h"
#include "nmpc_generator.h"
#include "mpc_generator.h"
#include "interpolation.h"
#include "kinematics.h"
#include "utils.h"

// Forward declare locations of configuration files.
std::string io_config;
std::string pg_config;
std::string ki_config;
std::string calib_file;

// Forward declare name of the robot.
std::string robot;

// Forward declare simulation.
bool simulation;

// Forward declare output location.
std::string out_loc;

// Forward declare BehaviouralCloning. This is actually the heart
// of the application. Within it, the pattern is generated,
// and the  inverse kinematics is computed. Also, images and velocities
// are recorded and stored.
class BehaviouralCloning : public yarp::os::BufferedPort<yarp::sig::Matrix>
{
    public:

        BehaviouralCloning(Eigen::VectorXd q_min, Eigen::VectorXd q_max, bool sim);

        ~BehaviouralCloning();

        using yarp::os::BufferedPort<yarp::sig::Matrix>::onRead;
        virtual void onRead(yarp::sig::Matrix& state);

        // State of this port.
        bool interrupted;

    public: // TEST.. change to private!

        // Building blocks of walking generation.
        NMPCGenerator pg_;
        Interpolation ip_;
        Kinematics ki_;

        // Current state of the robot.
        PatternGeneratorState pg_state_;

        Eigen::VectorXd   q_;
        Eigen::VectorXd  dq_;
        Eigen::VectorXd ddq_;
        Eigen::Vector3d com_pos_;
        Eigen::Vector3d com_vel_;
        Eigen::Vector3d com_acc_;

        // Extremal joint angles of the robot.
        Eigen::VectorXd q_min_;
        Eigen::VectorXd q_max_;

        // User input.
        Eigen::Vector3d vel_;

        // State of the robot on preview horizon.
        Eigen::MatrixXd traj_;
        Eigen::MatrixXd com_traj_;
        Eigen::MatrixXd lf_traj_;
        Eigen::MatrixXd rf_traj_;
        Eigen::MatrixXd q_traj_;

        // External velocity input and joint angle port.
        yarp::os::BufferedPort<yarp::sig::Vector> port_vel_;
        yarp::os::BufferedPort<yarp::sig::Vector> port_q_;
        yarp::os::BufferedPort<yarp::sig::Vector> port_lft_;
        yarp::os::BufferedPort<yarp::sig::Vector> port_rft_;

        // Mutex.
        yarp::os::Mutex mutex_;

        // Position initialized.
        RobotStatus robot_status_;
        bool initialized_;

        // Port to communicate the status.
        yarp::os::BufferedPort<yarp::os::Bottle> port_status_;

        // Force torque.
        bool simulation_;
        Eigen::MatrixXd ft_;
};


class StoreData : public yarp::os::RateThread
{
    public:
        StoreData(int period, std::vector<Part> parts, std::string calib_file, bool sim, std::string out_location);
        ~StoreData();

        inline const int& GetEpoch() const { return epoch_; };

    private:
        void run() override;
        void ProcessImages();

        // Ports to read velocities, images, and the current epoch.
        yarp::os::BufferedPort<yarp::sig::Vector> port_vel_;        
        std::map<std::string, yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb>>> ports_img_;
        yarp::os::BufferedPort<yarp::os::Bottle> port_epoch_;

        // Output images, and corresponding velocities.
        std::string out_location_; // location of txt file and images

        std::chrono::steady_clock::time_point start_time_; // time stamp and epoch for labeling output
        std::chrono::milliseconds time_stamp_;
        int epoch_;

        // Current velocity.
        Eigen::Vector3d vel_;

        // Parts.
        std::vector<Part> parts_;

        // Images of the cameras.
        std::map<std::string, yarp::sig::ImageOf<yarp::sig::PixelRgb>> imgs_;
        std::map<std::string, cv::Mat> imgs_cv_rgb_;
        std::map<std::string, cv::Mat> imgs_cv_gra_;

        // Stereo matching and weighted least square filter.
        cv::Ptr<cv::StereoBM> l_matcher_;
        cv::Ptr<cv::StereoMatcher> r_matcher_;
        cv::Ptr<cv::ximgproc::DisparityWLSFilter> wls_;

        // Disparity map.
        cv::Mat l_disp_; 
        cv::Mat r_disp_; 
        cv::Mat wls_disp_;

        // Position initialized.
        RobotStatus robot_status_;
        bool initialized_;

        // Port to communicate the status.
        yarp::os::BufferedPort<yarp::os::Bottle> port_status_;

        // Calibration.
        bool simulation_;
        bool calib_initialized_;

        cv::Mat R1_, R2_, P1_, P2_, Q_;
        cv::Mat K1_, K2_, R_;
        cv::Vec3d T_;
        cv::Mat D1_, D2_;

        cv::Mat lmapx_, lmapy_, rmapx_, rmapy_;
};


// Main application for heicub_walking.
int main(int argc, char *argv[]) {

    // Read user input to obtain locations of the configuration
    // files and the robot's name.
    yarp::os::Property params;
    params.fromCommand(argc, argv);

    if (params.check("io_config")) {
        io_config = params.find("io_config").asString();
    }
    else {
        std::cerr << "Please specify the location of the input output configurations file" << std::endl;
        std::cerr << "--io_config (e.g. ../libs/io_module/configs.yaml)" << std::endl;
        std::exit(1);
    }
    if (params.check("pg_config")) {
        pg_config = params.find("pg_config").asString();
    }
    else {
        std::cerr << "Please specify the location of the pattern generator configurations file" << std::endl;
        std::cerr << "--pg_config (e.g. ../libs/pattern_generator/configs.yaml)" << std::endl;
        std::exit(1);
    }
    if (params.check("ki_config")) {
        ki_config = params.find("ki_config").asString();
    }
    else {
        std::cerr << "Please specify the location of the kinematics configurations file" << std::endl;
        std::cerr << "--ki_config (e.g. ../libs/io_module/cam_stereo.yaml)" << std::endl;
        std::exit(1);
    }
    if (params.check("calib_file")) {
        calib_file = params.find("calib_file").asString();
    }
    else {
        std::cerr << "Please specify the location of the calibration file" << std::endl;
        std::cerr << "--calib_file (e.g. ../libs/kinematics/configs.yaml)" << std::endl;
        std::exit(1);
    }
    if (params.check("robot")) {
        robot = params.find("robot").asString();
    }
    else {
        std::cerr << "Please specify name of the robot" << std::endl;
        std::cerr << "--robot (e.g. icubGazeboSim)" << std::endl;
        std::exit(1);
    }
    if (params.check("out_loc")) {
        out_loc = params.find("out_loc").asString();
    }
    else
    {
        std::cerr << "Please specify output location" << std::endl;
        std::cerr << "--out_loc (e.g. ../out)" << std::endl;
        std::exit(1);
    }
    if (params.check("simulation")) {
        simulation = params.find("simulation").asBool();
    }
    else {
        std::cerr << "Please specify whether or not to run in simulation" << std::endl;
        std::cerr << "--simulation (e.g. true)" << std::endl;
        std::exit(1);
    }   

    // Set up the yarp network.
    yarp::os::Network yarp;

    // Reader and writer.
    // int period = YAML::LoadFile(pg_config)["t"].as<double>()*1e3;              // preview horizon
    int period = YAML::LoadFile(pg_config)["command_period"].as<double>()*1e3; // interpolation time resolution
    int period_cam = 100;

    ReadJoints rj(period, io_config, robot);
    WriteJoints wj(period, io_config, robot);
    ReadCameras rc(period_cam, io_config, robot);

    // Get the extremal angles for the joints. // TODO add some kind of min max init
    Eigen::VectorXd min = rj.GetMinAngles();
    Eigen::VectorXd max = rj.GetMaxAngles();

    // Process data, read from joints.
    BehaviouralCloning bc_port(min, max, simulation); 
    bc_port.open("/behavioural_cloning/nmpc_pattern_generator");

    StoreData sd(200, rc.GetParts(), calib_file, simulation, out_loc);

    // Connect reader to external commands (possibly ai thread).
    yarp::os::Network::connect("/reader/vel", "/behavioural_cloning/vel"); // send commands from terminal (reader.cpp) to this main
    yarp::os::Network::connect("/reader/robot_status", "/behavioural_cloning/robot_status"); // send robot status from keyreader to this main
    yarp::os::Network::connect("/reader/vel", "/store_data/vel"); // send commands from terminal (reader.cpp) to this main
    yarp::os::Network::connect("/reader/robot_status", "/store_data/robot_status"); // send robot status from keyreader to this main

    // Ports to read images and the current epoch.
    for (const auto& part : rc.GetParts()) {
        for (const auto& camera : part.cameras) {

            yarp::os::Network::connect("/read_cameras/" + camera, "/store_data/" + camera);
        }
    }
    yarp::os::Network::connect("/reader/epoch", "/store_data/epoch");

    // Put reader, processor, and writer together.
    yarp::os::Network::connect(rj.GetPortName(), "/behavioural_cloning/nmpc_pattern_generator");    // connect reader to BehaviouralCloning -> onRead gets called
    yarp::os::Network::connect("/behavioural_cloning/joint_angles", wj.GetPortName()); // connect to port_q of BehaviouralCloning
    yarp::os::Network::connect("/behavioural_cloning/robot_status", "/user_interface/robot_status"); // send robot status from keyreader to this main
    yarp::os::Network::connect("/write_joints/robot_status", "/user_interface/robot_status"); // send commands from writer.cpp to terminal
    yarp::os::Network::connect("/write_joints/robot_status", "/behavioural_cloning/robot_status"); // send commands from writer.cpp to this main

    // Read force torque.
    if (!simulation) {
    
        yarp::os::Network::connect("/wholeBodyDynamics/left_leg/cartesianEndEffectorWrench:o", "/behavioural_cloning/lft"); // read force torques from yarp to behavioural_cloning
        yarp::os::Network::connect("/wholeBodyDynamics/right_leg/cartesianEndEffectorWrench:o", "/behavioural_cloning/rft"); // read force torques from yarp to behavioural_cloning
    }

    // Start the read and write threads.
    rj.start();
    wj.start();
    rc.start();

    // Start the store data thread.
    sd.start();
    
    // Run program for a certain delay.
    while (!bc_port.interrupted) {

        // Run until port is disconnected.
        yarp::os::Time::delay(1e-1);
    }

    // Save trajectories.
    int ep = sd.GetEpoch();
    WriteCsv(out_loc + "/data/trajectories_epoch_" + std::to_string(ep) + ".csv", bc_port.ip_.GetTrajectories().transpose());

    if (!simulation) {

        WriteCsv(out_loc + "/data/force_torque_epoch_" + std::to_string(ep) + ".csv", bc_port.ft_.transpose());
    }

    // Stop reader and writer (on command later).
    bc_port.close();
    rj.stop();
    wj.stop();
    rc.stop();
    sd.stop();

    return 0;
}


// Implement BehaviouralCloning.
BehaviouralCloning::BehaviouralCloning(Eigen::VectorXd q_min, Eigen::VectorXd q_max, bool sim)
  : interrupted(false),

    pg_(pg_config),
    ip_(pg_), 
    ki_(ki_config),

      q_(ki_.GetQTraj().rows()),
     dq_(ki_.GetQTraj().rows()),
    ddq_(ki_.GetQTraj().rows()),

    // Extremal joint angles of the robot.
    q_min_(q_min),
    q_max_(q_max),
    
    // State of the robot on preview horizon.
    com_traj_(4, 1),//  ip_.GetTrajectoriesBuffer().cols()),
    lf_traj_(4, 1),// ip_.GetTrajectoriesBuffer().cols()),
    rf_traj_(4, 1),// ip_.GetTrajectoriesBuffer().cols()),
    
    // Position initialized.
    robot_status_(NOT_INITIALIZED),
    initialized_(false),
    
    simulation_(sim),
    ft_(12, 0) {

    // Pattern generator preparation.
    pg_.SetSecurityMargin(pg_.SecurityMarginX(), 
                          pg_.SecurityMarginY());

    // Set initial values.
    pg_state_ = {pg_.Ckx0(),
                 pg_.Cky0(),
                 pg_.Hcom(),
                 pg_.Fkx0(),
                 pg_.Fky0(),
                 pg_.Fkq0(),
                 pg_.CurrentSupport().foot,
                 pg_.Ckq0()};

    pg_.SetInitialValues(pg_state_);

    // Use callback to call onRead() method.
    useCallback();
    setCallbackLock(&mutex_);

    // Set initial velocity to zero.
    vel_.setZero();

    // Open port for velocity input.
    port_vel_.open("/behavioural_cloning/vel"); // open /behavioural_cloning/vel port to read velocity commands from terminal via reader.cpp
    port_q_.open("/behavioural_cloning/joint_angles"); // open /behavioural_cloning/joint_angles port to read joint angles from writer.cpp
    port_lft_.open("/behavioural_cloning/lft"); // open /user_controlled_walking/lft to read in force torque from yarp
    port_rft_.open("/behavioural_cloning/rft"); // open /user_controlled_walking/rft to read in force torque from yarp
    port_status_.open("/behavioural_cloning/robot_status"); // open /behavioural_cloning/robot_status to write status information to the terminal via reader.cpp

    ip_.StoreTrajectories(true);
}


BehaviouralCloning::~BehaviouralCloning() {

    // Close ports.
    port_vel_.close();
    port_q_.close();
    port_lft_.close();
    port_rft_.close();
    port_status_.close();
}


// Implement onRead() method.
void  BehaviouralCloning::onRead(yarp::sig::Matrix& state) {

    // Stop pattern generation on emergency stop.
    if (!yarp::os::Network::isConnected(port_status_.getName(), "/user_interface/robot_status")) {
        std::cout << "Quitting pattern generation on emergency stop." << std::endl;
        this->interrupt();
        interrupted = true;
    }

    // Lock callbacks during the computation.
    lockCallback();

    yarp::os::Bottle* bottle = port_status_.read(false);
    if (bottle != YARP_NULLPTR) {

        // Get initial position status.
        robot_status_ = RobotStatus(bottle->pop().asDict()->find("RobotStatus").asInt());
    }

    if (robot_status_ == STOPPING) {
        
        interrupted = true;
    }

    if (initialized_ && robot_status_ == INITIALIZED && !interrupted) {

        // Read the desired velocity and keep it unchanged if
        // no command arrives.
        yarp::sig::Vector* vel = port_vel_.read(false);
        if (vel != YARP_NULLPTR) {

            // Convert to Eigen.
            vel_ = yarp::eigen::toEigen(*vel);
        }

        // Set desired velocity.
        pg_.SetVelocityReference(vel_);

        // Solve QP.
        pg_.Solve();
        pg_.Simulate();
        traj_ = ip_.InterpolateStep();

        if (pg_.GetStatus() != qpOASES::SUCCESSFUL_RETURN) {

            // Communicate unfeasible qp.
            yarp::os::Bottle& bottle = port_status_.prepare();
            yarp::os::Property& dict = bottle.addDict();

            dict.put("ERROR", QP_INFEASIBLE);
            port_status_.write(); // write status to port which calls onRead() method of KeyReader or AppReader and is received by the app

            std::exit(1);
        }

        // Use forward kinematics to obtain the com feedback.
        q_ << ki_.GetQTraj().topRows(6).col(0), yarp::eigen::toEigen(state.getCol(0)).bottomRows(15);

        // Generate pattern with com feedback.
        pg_state_.com_x(0) = com_pos_(0);//, com_vel_(0), com_acc_(0);
        pg_state_.com_y(0) = com_pos_(1);//, com_vel_(1), com_acc_(1);
        pg_state_.com_z = com_pos_(2);

        pg_state_ = pg_.Update();
        pg_.SetInitialValues(pg_state_);

        // Check for correctness of inverse kinematics.
        if (!ki_.GetStatus()) {

            // Communicate inverse kinematics status.
            yarp::os::Bottle& bottle = port_status_.prepare();
            yarp::os::Property& dict = bottle.addDict();

            dict.put("Warning", IK_DID_NOT_CONVERGE);
            port_status_.write(); // write status to port which calls onRead() method of KeyReader or AppReader and is received by the app
        }

        // Check for hardware limits.
        bool limits = false;

        limits = limits && (q_traj_.array() < q_min_.array()).any();
        limits = limits && (q_traj_.array() > q_max_.array()).any();

        if (limits) {

            // Communicate hardware limits.
            yarp::os::Bottle& bottle = port_status_.prepare();
            yarp::os::Property& dict = bottle.addDict();

            dict.put("ERROR", HARDWARE_LIMITS);
            port_status_.write(); // write status to port which calls onRead() method of KeyReader or AppReader and is received by the app

            std::exit(1);
        }

        if (!simulation_) 
        {
            ft_.conservativeResize(ft_.rows(), ft_.cols() + ip_.GetTrajectoriesBuffer().cols());
        }

        for (int i = 0; i < traj_.cols(); i++)
        {
            com_traj_ << traj_(0, i),  traj_(3, i),  traj_(6, i),  traj_(7, i);
            lf_traj_ << traj_(13, i), traj_(14, i), traj_(15, i), traj_(16, i);
            rf_traj_ << traj_(17, i), traj_(18, i), traj_(19, i), traj_(20, i);  

            if (!simulation_) 
            {
            
                yarp::sig::Vector* lft = port_lft_.read();
                yarp::sig::Vector* rft = port_rft_.read();

                for (int j = 0; j < (*lft).size(); j++) {
                    
                    ft_(j, ft_.cols() - ip_.GetTrajectoriesBuffer().cols() + i) = (*lft)[j];
                    ft_(j + (*lft).size(), ft_.cols() - ip_.GetTrajectoriesBuffer().cols() + i) = (*rft)[j];
                }
            }

            ki_.Inverse(com_traj_, lf_traj_, rf_traj_);
            q_traj_ = ki_.GetQTraj().bottomRows(15);

            // Write joint angles to output port.
            yarp::sig::Vector data(q_traj_.rows(), q_traj_.cols());
            Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(data.data(), q_traj_.rows(), q_traj_.cols()) = q_traj_;

            port_q_.prepare() = data;
            port_q_.write();

            yarp::os::Time::delay(ip_.GetCommandPeriod()); // convert to seconds
        }
     }

    else if (!initialized_ && !interrupted) {

        // Initialize position with a good guess.
        Eigen::VectorXd q_init(21);
        q_init.setZero();
        
        q_init(2)  = 0.6;
        q_init(6)  = 0.54;
        q_init(9)  = -0.57;
        q_init(10) = -0.23;
        q_init(12) = 0.54;
        q_init(15) = -0.57;
        q_init(16) = -0.23;

        ki_.SetQInit(q_init);

        // Get desired initial state of the robot.
        traj_ = ip_.GetTrajectoriesBuffer().col(0);

        // Initialize inverse kinematics.
        com_traj_ << traj_.row(0),  traj_.row(3),  traj_.row(6),  traj_.row(7);
        lf_traj_  << traj_.row(13), traj_.row(14), traj_.row(15), traj_.row(16);
        rf_traj_  << traj_.row(17), traj_.row(18), traj_.row(19), traj_.row(20);

        ki_.Inverse(com_traj_, lf_traj_, rf_traj_);
        q_traj_ = ki_.GetQTraj().bottomRows(15).col(0);

        // Write joint angles to output port.
        yarp::sig::Vector data(q_traj_.rows(), q_traj_.cols());
        Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(data.data(), q_traj_.rows(), q_traj_.cols()) = q_traj_;

        port_q_.prepare() = data;
        port_q_.write();

        initialized_ = true;
    }

    // Unlock the callback.
    unlockCallback();
}


StoreData::StoreData(int period, std::vector<Part> parts, std::string calib_file, bool sim, std::string out_location) 
    : RateThread(period),
    
      out_location_(out_location),

	  start_time_(std::chrono::steady_clock::now()),
	  time_stamp_(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start_time_)),
      epoch_(0),
      parts_(parts),

      // Position initialized.
      robot_status_(NOT_INITIALIZED),
      initialized_(false)
{
    // Open port for velocity input and robot status.
    port_vel_.open("/store_data/vel"); // open /store_data/vel port to read velocity commands from terminal via reader.cpp
    port_status_.open("/store_data/robot_status"); // open /store_data/robot_status to write status information to the terminal via reader.cpp

    // Ports to read images and the current epoch.
    for (const auto& part : parts_) {
        for (const auto& camera : part.cameras) {

            ports_img_[camera].open("/store_data/" + camera);
        }
    }

    port_epoch_.open("/store_data/epoch");
 
    // Set initial velocity to zero.
    vel_.setZero();

    // Stereo matching and weighted least square filter.
    l_matcher_ = cv::StereoBM::create(32, 13);
    r_matcher_ = cv::ximgproc::createRightMatcher(l_matcher_);
    wls_ = cv::ximgproc::createDisparityWLSFilter(l_matcher_);

    wls_->setLambda(1.e4);
    wls_->setSigmaColor(1.);

    // Calibration.
    simulation_ = sim;
    calib_initialized_ = false;

    cv::FileStorage fs1(calib_file, cv::FileStorage::READ);
    fs1["K1"] >> K1_;
    fs1["K2"] >> K2_;
    fs1["D1"] >> D1_;
    fs1["D2"] >> D2_;
    fs1["R"] >> R_;
    fs1["T"] >> T_;

    fs1["R1"] >> R1_;
    fs1["R2"] >> R2_;
    fs1["P1"] >> P1_;
    fs1["P2"] >> P2_;
    fs1["Q"] >> Q_;
}


void StoreData::run() {

    yarp::os::Bottle* bottle = port_status_.read(false);
    if (bottle != YARP_NULLPTR) {

        // Get initial position status.
        robot_status_ = RobotStatus(bottle->pop().asDict()->find("RobotStatus").asInt());
    }

    if (true){//robot_status_ == INITIALIZED) {

        // Read the current demanded velocity.
        yarp::sig::Vector* vel = port_vel_.read(false);
        if (vel != YARP_NULLPTR) {

            // Convert to Eigen.
            vel_ = yarp::eigen::toEigen(*vel);
        }

        // Read current images, compute depth image, and save them with corresponding velocity and time stamp.
        ProcessImages();
    }
}


StoreData::~StoreData() {
        
    // Close ports.
    port_vel_.close();

    for (const auto& part : parts_) {
        for (const auto& camera : part.cameras) {

            ports_img_[camera].close();
        }
    }

    port_epoch_.close();
    port_status_.close();
}


void StoreData::ProcessImages() {

    // Read the camera images.
    bool null = true;

    for (const auto& part : parts_) {
        for (const auto& camera : part.cameras) {

            yarp::sig::ImageOf<yarp::sig::PixelRgb>* img = ports_img_[camera].read(false);
            if (img != YARP_NULLPTR) {

                // Convert the images to a format that OpenCV uses.
                imgs_cv_rgb_[camera] = cv::cvarrToMat(img->getIplImage());

                if (calib_initialized_ && !simulation_) {

                    if (camera == "left") {

                        cv::remap(imgs_cv_rgb_[camera], imgs_cv_rgb_[camera], lmapx_, lmapy_, cv::INTER_LINEAR);
                    }
                    else {

                        cv::remap(imgs_cv_rgb_[camera], imgs_cv_rgb_[camera], rmapx_, rmapy_, cv::INTER_LINEAR);
                    }
                }

                // Convert to gray image.
                cv::cvtColor(imgs_cv_rgb_[camera], imgs_cv_gra_[camera], cv::COLOR_BGR2GRAY);

                null = false;
            }
        }
    }

    if (!null) {

        if (calib_initialized_) {

            // Determine disparity.
            l_matcher_->compute(imgs_cv_gra_[parts_[0].cameras[0]], imgs_cv_gra_[parts_[0].cameras[1]], l_disp_);

            r_matcher_->compute(imgs_cv_gra_[parts_[0].cameras[1]], imgs_cv_gra_[parts_[0].cameras[0]], r_disp_);

            // Perform weighted least squares filtering.
            wls_->filter(l_disp_, imgs_cv_gra_[parts_[0].cameras[0]], wls_disp_, r_disp_);

            cv::ximgproc::getDisparityVis(wls_disp_, wls_disp_, 1);
            cv::normalize(wls_disp_, wls_disp_, 0, 255, CV_MINMAX, CV_8U);

            // Set the time stamp.
            time_stamp_ = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start_time_);

            yarp::os::Bottle* epoch = port_epoch_.read(false);
            if (epoch != YARP_NULLPTR) {
                epoch_ = epoch->get(0).asInt();
            }

            // Record images with time stamp.
            // Fill stringstream with preceeding zeros.
            std::ostringstream ss;
            ss << std::setw(8) << std::setfill('0') << std::to_string(time_stamp_.count());

            // Track locations of stored images and corresponding velocity.
            std::ofstream txt(out_location_ + "/log.txt", std::ios_base::app);

            std::string loc = out_location_ + "/data/img/left_epoch_" + std::to_string(epoch_) + "_" + ss.str() + ".jpg";
            cv::imwrite(loc, imgs_cv_rgb_["left"]);
            txt << "data/img/left_epoch_" + std::to_string(epoch_) + "_" + ss.str() + ".jpg" + ", ";


            loc = out_location_ + "/data/img/wls_disp_epoch_" + std::to_string(epoch_) + "_" + ss.str() + ".jpg";
            cv::imwrite(loc, wls_disp_);
            txt << "data/img/wls_disp_epoch_" + std::to_string(epoch_) + "_" + ss.str() + ".jpg" + ", ";

            int i = 0;
            while (i < vel_.size()-1) {
                txt << vel_[i] << ", ";
                i++;
            }
            txt << vel_[vel_.size()-1] << "\n";
        }
        else {

            if (!simulation_) {
                cv::initUndistortRectifyMap(K1_, D1_, R1_, P1_, imgs_cv_rgb_["left"].size(), CV_32F, lmapx_, lmapy_);
                cv::initUndistortRectifyMap(K2_, D2_, R2_, P2_, imgs_cv_rgb_["right"].size(), CV_32F, rmapx_, rmapy_);
            }

            calib_initialized_ = true;
        }
    }
}

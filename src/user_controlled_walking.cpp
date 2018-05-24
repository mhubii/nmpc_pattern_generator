#include <yarp/os/all.h>
#include <yarp/os/all.h>
#include <Eigen/Core>
#include <rbdl/rbdl.h>

#include "reader.h"
#include "writer.h"
#include "nmpc_generator.h"
#include "interpolation.h"
#include "kinematics.h"
#include "utils.h"

// Forward declare locations of configuration files.
std::string io_config;
std::string pg_config;
std::string ki_config;

// Forward declare name of the robot.
std::string robot;

// Define possible errors.
enum Errors { NO_CONNECTION, QP_INFEASIBLE, HARDWARE_LIMITS };

// Forward declare WalkingProcessor. This is actually the heart
// of the application. Within it, the pattern is generated,
// and the dynamic filter as well as the inverse kinematics are
// computed.
class WalkingProcessor : public yarp::os::BufferedPort<yarp::sig::Matrix>
{
    public:

        WalkingProcessor();

        ~WalkingProcessor();

        using yarp::os::BufferedPort<yarp::sig::Matrix>::onRead;
        virtual void onRead(yarp::sig::Matrix& state);

        // Setter.
        inline void SetIntitialPositionStatus(InitialPositionStatus pos) { init_pos_status_ = pos; };

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

        // Mutex.
        yarp::os::Mutex mutex_;

        // Position initialized.
        InitialPositionStatus init_pos_status_;
        bool initialized_;

        // Port to read initial position status.
        yarp::os::BufferedPort<yarp::os::Bottle> port_init_pos_status_;

        // TEST
        // Eigen::MatrixXd qq;
        // int count;
        // TEST END
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
        std::cerr << "--ki_config (e.g. ../libs/kinematics/configs.yaml)" << std::endl;
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

    // Set up the yarp network.
    yarp::os::Network yarp;

    // Reader and writer.
    int period = 10;

    ReadJoints rj(period, io_config);
    WriteJoints wj(period, io_config);

    // Process data, read from joints.
    WalkingProcessor pg_port; 
    pg_port.open("/test/port");

    // Connect reader to external commands (possibly ai thread).
    yarp::os::Network::connect("/vel/command", "/vel");

    // Put reader, processor, and writer together.
    yarp::os::Network::connect(rj.GetPortName(), "/test/port");
    yarp::os::Network::connect("/joint_angles", wj.GetPortName());
    yarp::os::Network::connect("/client_write/init_pos_status", "/key_reader/commands");
    yarp::os::Network::connect("/client_write/init_pos_status", "/walking_processor/commands");

    // Start the read and write threads.
    rj.start();
    wj.start();
    
    // Run program for a certain delay.
    yarp::os::Time::delay(20);

    // TEST
    WriteCsv("test.csv", pg_port.ip_.GetTrajectories().transpose());
    // TEST END

    // Stop reader and writer (on command later).
    pg_port.close();
    rj.stop();
    wj.stop();

    // system("pause");

    return 0;
}


// Implement WalkingProcessor.
WalkingProcessor::WalkingProcessor()
  : pg_(pg_config),
    ip_(pg_), 
    ki_(ki_config),

      q_(ki_.GetQTraj().rows()),
     dq_(ki_.GetQTraj().rows()),
    ddq_(ki_.GetQTraj().rows()),
    
    // State of the robot on preview horizon.
    com_traj_(4, 1),// ip_.GetTrajectoriesBuffer().cols()),
    lf_traj_(4, 1),// ip_.GetTrajectoriesBuffer().cols()),
    rf_traj_(4, 1),// ip_.GetTrajectoriesBuffer().cols()),
    
    // Position initialized.
    init_pos_status_(NOT_STARTED),
    initialized_(false) {

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
    port_vel_.open("/vel");
    port_q_.open("/joint_angles");
    port_init_pos_status_.open("/walking_processor/commands");

    // TEST
    ip_.StoreTrajectories(true);
    // TEST END
    // TEST
    // qq = ReadCsv<Eigen::MatrixXd>("/home/martin/Documents/heicub_walking/build/example_nmpc_generator_interpolated_results.csv").transpose();
    // count = 0;
    // TEST END
}


WalkingProcessor::~WalkingProcessor() {

    // Close ports.
    port_vel_.close();
    port_q_.close();
    port_init_pos_status_.close();
}


// Implement onRead() method.
void  WalkingProcessor::onRead(yarp::sig::Matrix& state) {
    
    // Lock callbacks during the computation.
    lockCallback();

    yarp::os::Bottle* bottle = port_init_pos_status_.read(false);
    if (bottle != YARP_NULLPTR) {

        // Get initial position status.
        init_pos_status_ = InitialPositionStatus(bottle->pop().asDict()->find("InitialPositionStatus").asInt());
    }

    if (initialized_ && init_pos_status_ == DONE) {

        // // Read the desired velocity and keep it unchanged if
        // // no command arrives.
        // yarp::sig::Vector* vel = port_vel_.read(false);
        // if (vel != YARP_NULLPTR) {

        //     // Convert to Eigen.
        //     vel_ = yarp::eigen::toEigen(*vel);
        // }

        // // Use forward kinematics to obtain the com feedback.
        // q_ << ki_.GetQTraj().topRows(6).col(0), yarp::eigen::toEigen(state.getCol(0)).bottomRows(15);
        // dq_.bottomRows(15) = yarp::eigen::toEigen(state.getCol(1));
        // ddq_.bottomRows(15) = yarp::eigen::toEigen(state.getCol(2));

        // ki_.Forward(q_, dq_, ddq_);
        // com_pos_ = ki_.GetComPos();
        // com_vel_ = ki_.GetComVel();
        // com_acc_ = ki_.GetComAcc();

        // Generate pattern with com feedback.
        // pg_state_ = pg_.Update();
        pg_state_.com_x << com_pos_(0), com_vel_(0), com_acc_(0);
        pg_state_.com_y << com_pos_(1), com_vel_(1), com_acc_(1);
        pg_state_.com_z = com_pos_(2);
        
        pg_.SetInitialValues(pg_state_);

        // // Set desired velocity.
        // pg_.SetVelocityReference(vel_);

        // // Solve QP.
        // pg_.Solve();
        // pg_.Simulate();

        // TEST
        // Set reference velocities.
        vel_ << 0.01, 0., 0.;
        pg_.SetVelocityReference(vel_);

        // pg_.SetInitialValues(pg_state_);

        // Solve QP.
        pg_.Solve();
        pg_.Simulate();
        ip_.Interpolate();
        traj_ = ip_.GetTrajectoriesBuffer().col(0);

        // Initial value embedding by internal states and simulation.
        pg_state_ = pg_.Update();
        // pg_.SetInitialValues(pg_state_);
        

        // traj_ = qq.col(count);

        // // Inverse kinematics.
        // com_traj_ << traj_.row(0),  traj_.row(3),  traj_.row(6),  traj_.row(7);
        // lf_traj_  << traj_.row(13), traj_.row(14), traj_.row(15), traj_.row(16);
        // rf_traj_  << traj_.row(17), traj_.row(18), traj_.row(19), traj_.row(20);

        // ki_.Inverse(com_traj_, lf_traj_, rf_traj_);
        // q_traj_ = ki_.GetQTraj().bottomRows(15).col(0); // TODO: there must be a mistake when the data vector gets read out.

        // yarp::sig::Vector data(q_traj_.rows(), q_traj_.cols());
        // Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(data.data(), q_traj_.rows(), q_traj_.cols()) = q_traj_;
        // port_q_.prepare() = data;
        // port_q_.write();

        // count++;
        // TEST END

        // // Interpolate results.
        // ip_.Interpolate();
        // traj_ = ip_.GetTrajectoriesBuffer().col(0);

        // Inverse kinematics.
        com_traj_ << traj_.row(0),  traj_.row(3),  traj_.row(6),  traj_.row(7);
        lf_traj_  << traj_.row(13), traj_.row(14), traj_.row(15), traj_.row(16);
        rf_traj_  << traj_.row(17), traj_.row(18), traj_.row(19), traj_.row(20);

        ki_.Inverse(com_traj_, lf_traj_, rf_traj_);
        q_traj_ = ki_.GetQTraj().bottomRows(15);

        // Write joint angles to output port.
        yarp::sig::Vector data(q_traj_.rows(), q_traj_.cols());
        Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(data.data(), q_traj_.rows(), q_traj_.cols()) = q_traj_;

        port_q_.prepare() = data;
        port_q_.write();
    }

    else if (!initialized_) {

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

#include <yarp/os/all.h>
#include <yarp/os/all.h>
#include <Eigen/Core>
#include <rbdl/rbdl.h>

#include "reader.h"
#include "writer.h"
#include "nmpc_generator.h"
#include "interpolation.h"
#include "kinematics.h"

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

    private:

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
        yarp::os::BufferedPort<yarp::sig::Matrix> port_q_;

        // Mutex.
        yarp::os::Mutex mutex_;
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
    int period = 1000;

    ReadJoints rj(period);
    rj.start();

    // WriteJoints wj(period);
    // wj.start();

    // Process data, read from joints.
    WalkingProcessor pg_port; 
    pg_port.open("/test/port");

    // Connect reader to external commands (possibly ai thread).
    yarp::os::Network::connect("/vel/command", "/vel");

    // Put reader, processor, and writer together.
    yarp::os::Network::connect(rj.GetPortName(), "/test/port");
    // yarp::os::Network::connect("/joint_angles", wj.GetPortName());
    
    // Run program for a certain delay.
    yarp::os::Time::delay(10);

    // Stop reader and writer (on command later).
    pg_port.close();
    rj.stop();
    // wj.stop();

    return 0;
}


// Implement WalkingProcessor.
WalkingProcessor::WalkingProcessor()
  : pg_(pg_config),
    ip_(pg_), 
    ki_(ki_config),
    
    // State of the robot on preview horizon.
    com_traj_(4, ip_.GetTrajectoriesBuffer().cols()),
    lf_traj_(4, ip_.GetTrajectoriesBuffer().cols()),
    rf_traj_(4, ip_.GetTrajectoriesBuffer().cols()) {

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

    // Open port for velocity input.
    port_vel_.open("/vel");
    port_q_.open("/joint_angles");
}


WalkingProcessor::~WalkingProcessor() {

    // Close ports.
    port_vel_.close();
    port_q_.close();
}


// Implement onRead() method.
void  WalkingProcessor::onRead(yarp::sig::Matrix& state) {
    
    // Lock callbacks during the computation.
    lockCallback();

    // Read the desired velocity.
    yarp::sig::Vector* vel = port_vel_.read();
    vel_ = yarp::eigen::toEigen(*vel);

    // Use forward kinematics to obtain the com feedback.
      q_ = yarp::eigen::toEigen(state.getCol(0));
     dq_ = yarp::eigen::toEigen(state.getCol(1));
    ddq_ = yarp::eigen::toEigen(state.getCol(2));

    ki_.Forward(q_, dq_, ddq_);
    com_pos_ = ki_.GetComPos();
    com_vel_ = ki_.GetComVel();
    com_acc_ = ki_.GetComAcc();

    // Generate pattern with com feedback.
    pg_state_ = pg_.Update();
    pg_state_.com_x << com_pos_(0), com_vel_(0), com_acc_(0);
    pg_state_.com_y << com_pos_(1), com_vel_(1), com_acc_(1);
    pg_state_.com_z = com_pos_(2);
    
    pg_.SetInitialValues(pg_state_);

    // Set desired velocity.
    pg_.SetVelocityReference(vel_);

    // Solve QP.
    pg_.Solve();
    pg_.Simulate();

    // Interpolate results.
    ip_.Interpolate();
    traj_ = ip_.GetTrajectoriesBuffer();

    // Inverse kinematics.
    com_traj_ << traj_.row(0),  traj_.row(3),  traj_.row(6),  traj_.row(7);
    lf_traj_  << traj_.row(13), traj_.row(14), traj_.row(15), traj_.row(16);
    rf_traj_  << traj_.row(17), traj_.row(18), traj_.row(19), traj_.row(20);

    ki_.Inverse(com_traj_, lf_traj_, rf_traj_);
    q_traj_ = ki_.GetQTraj().bottomRows(15);

    // Write joint angles to output port.
    yarp::sig::Matrix data(q_traj_.cols(), q_traj_.rows());
    Eigen::Map<Eigen::MatrixXd>(data.data(), q_traj_.rows(), q_traj_.cols()) = q_traj_;

    port_q_.prepare() = data;
    port_q_.write();

    // Unlock the callback.
    unlockCallback();
}

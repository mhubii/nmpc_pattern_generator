#include <yarp/os/all.h>
#include <yarp/os/all.h>
#include <Eigen/Core>
#include <rbdl/rbdl.h>
#include <qpOASES.hpp>

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


    // Generate offline walking trajectory.
    NMPCGenerator pg(pg_config);


    // Pattern generator preparation.
    pg.SetSecurityMargin(pg.SecurityMarginX(), 
                         pg.SecurityMarginY());


    // Set initial values.
    PatternGeneratorState pg_state = {pg.Ckx0(),
                                      pg.Cky0(),
                                      pg.Hcom(),
                                      pg.Fkx0(),
                                      pg.Fky0(),
                                      pg.Fkq0(),
                                      pg.CurrentSupport().foot,
                                      pg.Ckq0()};


    pg.SetInitialValues(pg_state);
    Interpolation ip(pg);
    ip.StoreTrajectories(true);
    Eigen::Vector3d vel(0.1, 0., 0.);


    // Pattern generator event loop.
    for (int i = 0; i < 2000; i++) {
        std::cout << "Iteration: " << i << std::endl;


        // Change reference velocities.
        if (50 <= i && i < 1500) {
            vel << 0.1, 0., 0.;
        }
        else if (150 <= i && i < 2000) {
            vel << 0., 0., 0.;
        }

        // Set reference velocities.
        pg.SetVelocityReference(vel);


        // Solve QP.
        pg.Solve();
        pg.Simulate();
        // ip.InterpolateStep();

        auto inter = ip.InterpolateStep();
        
        auto preview = int(ip.preview_intervals_);
        auto dt = pg.CpuTime();
        auto com_x = inter.block(0, preview, 3, 1);// initial values not final
        auto com_y = inter.block(3, preview, 3, 1);
        auto lfx = inter(13, preview);
        auto lfy = inter(14, preview);
        auto rfx = inter(17, preview);
        auto rfy = inter(18, preview);

        // Initial value embedding by internal states and simulation.
        pg_state = pg.Update(dt);
        pg_state.com_x = com_x;
        pg_state.com_y = com_y;
        if (pg.current_support_.foot == "left") {
            pg_state.foot_x = lfx;
            pg_state.foot_y = lfy;
        }
        else {
            pg_state.foot_x = rfx;
            pg_state.foot_y = rfy;
        }

        // Initial value embedding by internal states and simulation.
        // pg_state = pg.Update();
        pg.SetInitialValues(pg_state);
    }

    // Set up the yarp network.
    yarp::os::Network yarp;

    // Kinematics.
    Kinematics ki(ki_config);

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

    ki.SetQInit(q_init);

    // Get desired initial state of the robot.
    Eigen::MatrixXd traj = ip.GetTrajectories();

    // Initialize inverse kinematics.
    Eigen::MatrixXd com_traj(4, 1);     
    Eigen::MatrixXd lf_traj(4, 1);  
    Eigen::MatrixXd rf_traj(4, 1);

    com_traj << traj(0, 0),  traj(3, 0),  traj(6, 0),  traj(7, 0);
    lf_traj << traj(13, 0), traj(14, 0), traj(15, 0), traj(16, 0);
    rf_traj << traj(17, 0), traj(18, 0), traj(19, 0), traj(20, 0);  

    ki.Inverse(com_traj, lf_traj, rf_traj);
    Eigen::MatrixXd q_traj = ki.GetQTraj().bottomRows(15);

    // Write joint angles to output port.
    yarp::sig::Vector data(q_traj.rows(), 1);
    Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(data.data(), q_traj.rows(), 1) = q_traj.col(0);

    // Writer.
    int period = int(ip.GetCommandPeriod()*1000); // convert to ms

    yarp::os::BufferedPort<yarp::sig::Vector> port;
    port.open("/joint_angles");
    WriteJoints wj(period, io_config);

    // Put writer and offline rate thread together.
    yarp::os::Network::connect("/joint_angles", wj.GetPortName());

    wj.start();

    port.prepare() = data;
    port.write();

    // Get the status of the initialzation.
    yarp::os::BufferedPort<yarp::os::Bottle> port_status;
    port_status.open("/walking_processor/commands");
    yarp::os::Network::connect("/write_joints/robot_status", "/walking_processor/commands"); // /client_write/robot_status is written to by writer

    RobotStatus robot_status = INITIALIZING;

    while (robot_status != INITIALIZED) {
        
        // Stop the program until the thread is initialized. 
        yarp::os::Bottle* bottle = port_status.read(false);
        if (bottle != YARP_NULLPTR) {

            // Get initial position status.
            robot_status = RobotStatus(bottle->pop().asDict()->find("RobotStatus").asInt());
        }
    }

    // Write data to port.
    for (int i = 1; i < traj.cols(); i++) {
        yarp::os::Time::delay(double(period)/1000.); // convert to seconds
        std::cout << i << std::endl;

        com_traj << traj(0, i),  traj(3, i),  traj(6, i),  traj(7, i);
        lf_traj << traj(13, i), traj(14, i), traj(15, i), traj(16, i);
        rf_traj << traj(17, i), traj(18, i), traj(19, i), traj(20, i);  

        ki.Inverse(com_traj, lf_traj, rf_traj);
        q_traj = ki.GetQTraj().bottomRows(15);

        // Write joint angles to output port.
        Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(data.data(), q_traj.rows(), 1) = q_traj.col(0);

        port.prepare() = data;
        port.write();
    }

    // Store data.
    WriteCsv("offline_walking.csv", ip.GetTrajectories().transpose());

    // Stop writer.
    port.close();
    wj.stop();

    // Close status port.
    port_status.close();

    return 0;
}

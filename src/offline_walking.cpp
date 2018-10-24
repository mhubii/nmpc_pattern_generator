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


// Offline RateThread.
class OfflineRateThread : public yarp::os::RateThread
{
    public:

        OfflineRateThread(int period)
          : RateThread(period) {
            // traj_(traj),
            // count(0) {
        
            port_.open("joint_angles");
        };

        ~OfflineRateThread() {
        
            port_.close();
        };

        // Getter.
        inline const std::string GetPortName() const { return port_.getName(); };

    private:

        // Methods to be implemented for RateThread.
        virtual void run() {
            
            if (count <= traj_.cols()) {

                // Send read data to a port.
                yarp::sig::Vector data(traj_.rows(), 1);
                Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(data.data(), traj_.rows(), 1) = traj_.col(count);

                port_.prepare() = data;
                port_.write();

                // Update counter.
                count++;
            }

            else {

                // Stop rate thread.
                this->stop();
            }
        };

        // Trajectory.
        Eigen::MatrixXd traj_;

        // To to write joint angles to.
        yarp::os::BufferedPort<yarp::sig::Vector> port_;

        // Internal counter.
        int count;
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
    Eigen::Vector3d vel(0.02, 0., 0.);


    // Pattern generator event loop.
    for (int i = 0; i < 200; i++) {
        std::cout << "Iteration: " << i << std::endl;


        // // Change reference velocities.
        // if (25 <= i && i < 50) {
        //     velocity_reference << 0.1, 0., 0.1;
        // }
        // else if (50 <= i && i < 150) {
        //     velocity_reference << 0.1, 0.1, 0.1;
        // }
        // else if (150 <= i && i < 200) {
        //     velocity_reference << 0., 0., 0.;
        // }


        // Set reference velocities.
        pg.SetVelocityReference(vel);


        // Solve QP.
        pg.Solve();
        pg.Simulate();
        ip.InterpolateStep();


        // Initial value embedding by internal states and simulation.
        pg_state = pg.Update();
        pg.SetInitialValues(pg_state);
    }


    // Set up the yarp network.
    yarp::os::Network yarp;


    // Writer.
    int period = ip.GetCommandPeriod();

    OfflineRateThread or(period, ip.GetTrajectories());
    // WriteJoints wj(period, io_config);


    // // Put writer and offline rate thread together.
    // yarp::os::Network::connect(or.GetPortName(), wj.GetPortName());


    // // Start the write thread.
    // or.start();
    // wj.start();
   
    // while (or.isRunning()) {

    //     // Stop the program until the thread is released.
    // }

    // // Store data.
    // WriteCsv("offline_walking.csv", ip.GetTrajectories().transpose());

    // // Stop writer.
    // wj.stop();

    return 0;
}

#include <stdio.h>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/eigen/Eigen.h>
#include <Eigen/Dense>
#include <iostream>

#include "reader.h"
#include "writer.h"

int main(int argc, char * argv[]) {
    
    // Set up yarp network.
    yarp::os::Network yarp;

    // Set up writer.
    int period = 100;
    WriteJoints wj(period);
    wj.start();

    // Initialize position with a good guess.
    Eigen::MatrixXd q_init(21, 3);
    q_init.setZero();

    q_init(2, 0)  = 0.6;
    q_init(6, 0)  = 0.54;
    q_init(9, 0)  = -0.57;
    q_init(10, 0) = -0.23;
    q_init(12, 0) = 0.54;
    q_init(15, 0) = -0.57;
    q_init(16, 0) = -0.23;

    Eigen::MatrixXd q_traj = q_init.bottomRows(15);
    std::cout << q_traj.col(0) << std::endl;

    // Write joint angles to output port.
    yarp::sig::Matrix data(q_traj.rows(), q_traj.cols());
    Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(data.data(), q_traj.rows(), q_traj.cols()) = q_traj;

    // Create /joints/read port.
    yarp::os::BufferedPort<yarp::sig::Matrix> port_q;

    port_q.open("/joints/read");

    yarp::os::Network::connect("/joints/read", "/joints/write");

    while (wj.GetInitialPositionStatus() != DONE) {
        port_q.prepare() = data;
        port_q.write();
    }

    wj.stop();

    return 0;
}
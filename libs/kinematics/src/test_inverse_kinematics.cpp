#include "inverse_kinematics.h"
#include "utils.h"

#include <Eigen/Core>
#include <iostream>

int main() {
    
    // Initialize inverse kinematics.
    const std::string config_file_loc = "../libs/kinematics/configs.yaml";

    InverseKinematics ik(config_file_loc);

    // Load generated pattern. The pattern is stored in the order:
    //           com                    zmp       lf         rf
    // x dx ddx y dy ddy z q dq ddq    x y z    x y z q    x y z q
    Eigen::MatrixXd pattern = ReadCsv<Eigen::MatrixXd>("example_nmpc_generator_interpolated_results.csv").transpose();

    Eigen::MatrixXd com_traj(4, pattern.cols());
    Eigen::MatrixXd lf_traj(4, pattern.cols());
    Eigen::MatrixXd rf_traj(4, pattern.cols());

    // Initial position.
    Eigen::VectorXd q_init(21);
    q_init.setZero();
    
    q_init(2)  = 0.6;
    q_init(6)  = 0.54;
    q_init(9)  = -0.57;
    q_init(10) = -0.23;
    q_init(12) = 0.54;
    q_init(15) = -0.57;
    q_init(16) = -0.23;

    com_traj << pattern.row(0), pattern.row(3), pattern.row(6), pattern.row(7);
    lf_traj  << pattern.row(13), pattern.row(14), pattern.row(15), pattern.row(16);
    rf_traj  << pattern.row(17), pattern.row(18), pattern.row(19), pattern.row(20);

    ik.Invert(q_init, com_traj, lf_traj, rf_traj);

    // Save inverted trajectory.
    Eigen::MatrixXd q_traj = ik.GetQInit().transpose().rightCols(15);

    WriteCsv("joint_angle_trajectories.csv", q_traj);

}
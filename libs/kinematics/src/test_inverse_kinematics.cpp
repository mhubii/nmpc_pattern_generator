#include "kinematics.h"
#include "utils.h"

#include <Eigen/Core>
#include <iostream>

int main() {
    
    // Initialize inverse kinematics.
    const std::string config_file_loc = "../libs/kinematics/configs.yaml";

    Kinematics k(config_file_loc);

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

    k.SetQInit(q_init);

    com_traj << pattern.row(0), pattern.row(3), pattern.row(6), pattern.row(7);
    lf_traj  << pattern.row(13), pattern.row(14), pattern.row(15), pattern.row(16);
    rf_traj  << pattern.row(17), pattern.row(18), pattern.row(19), pattern.row(20);

    Eigen::MatrixXd q_traj(pattern.cols(), 15);

    Eigen::MatrixXd com_cur;
    Eigen::MatrixXd lf_cur;
    Eigen::MatrixXd rf_cur;

    for (int i = 0; i < pattern.cols(); i++) {
        
        // Get current trajectory time step.
        com_cur = com_traj.col(i);
        lf_cur = lf_traj.col(i);
        rf_cur = rf_traj.col(i);

        // Perform inverse kinematics.
        k.Inverse(com_cur, lf_cur, rf_cur);

        // Get results.
        q_traj.row(i) = k.GetQTraj().transpose().rightCols(15);
    }

    // Save inverted trajectory.
    WriteCsv("joint_angle_trajectories.csv", q_traj);
}
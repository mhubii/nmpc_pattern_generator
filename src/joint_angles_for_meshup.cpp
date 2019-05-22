#include <iostream>

#include "nmpc_generator.h"
#include "interpolation.h"
#include "kinematics.h"

int main() {
    // Initialize pattern generator.
    const std::string nmpc_config_file_loc = "../../libs/pattern_generator/configs_hrp2.yaml";

    NMPCGenerator nmpc(nmpc_config_file_loc);

    // Initialize kinematics.
    const std::string kinematics_config_file_loc = "../../libs/kinematics/configs.yaml";

    Kinematics ki(kinematics_config_file_loc);

    // Joint angles to store.
    Eigen::MatrixXd q_traj(1, 21);
    q_traj = Eigen::MatrixXd::Zero(1, 21); // degrees of freedom of robot
    Eigen::VectorXd time(1);
    time = Eigen::VectorXd::Zero(1);
    double t = 0.;


    // State of the robot.
    Eigen::MatrixXd com_traj(4, 1);
    Eigen::MatrixXd lf_traj(4, 1);
    Eigen::MatrixXd rf_traj(4, 1);

    // Pattern generator preparation.
    nmpc.SetSecurityMargin(nmpc.SecurityMarginX(), 
                           nmpc.SecurityMarginY());

    // Set initial values.
    PatternGeneratorState pg_state = {nmpc.Ckx0(),
                                      nmpc.Cky0(),
                                      nmpc.Hcom(),
                                      nmpc.Fkx0(),
                                      nmpc.Fky0(),
                                      nmpc.Fkq0(),
                                      nmpc.CurrentSupport().foot,
                                      nmpc.Ckq0()};

    nmpc.SetInitialValues(pg_state);
    Interpolation interpol_nmpc(nmpc);
    interpol_nmpc.StoreTrajectories(true);
    Eigen::Vector3d velocity_reference(0.01, 0., 0.);


    // Pattern generator event loop.
    for (int i = 0; i < 100; i++) {
        std::cout << "Iteration: " << i << std::endl;


        // Set reference velocities.
        nmpc.SetVelocityReference(velocity_reference);


        // Solve QP.
        nmpc.Solve();
        nmpc.Simulate();
        Eigen::MatrixXd traj = interpol_nmpc.InterpolateStep();


        // Initial value embedding by internal states and simulation.
        pg_state = nmpc.Update();
        nmpc.SetInitialValues(pg_state);

        // Inverse kinematics.
/*
        q_traj.conservativeResize(q_traj.rows() + traj.cols(), q_traj.cols());
        time.conservativeResize(time.rows() + traj.cols(), time.cols());

        for (int j = 0; j < traj.cols(); j++)
        {
            com_traj << traj(0, j),  traj(3, j),  traj(6, j),  traj(7, j);
            lf_traj << traj(13, j), traj(14, j), traj(15, j), traj(16, j);
            rf_traj << traj(17, j), traj(18, j), traj(19, j), traj(20, j);  

            ki.Inverse(com_traj, lf_traj, rf_traj);
            q_traj.row(i*traj.cols() + j) = ki.GetQTraj().transpose();

            t += interpol_nmpc.GetCommandPeriod();
            time(i*traj.cols() + j) = t;
        }   
*/     
    }

/*
    // Save results to .csv file and format it for meshup to read it properly.
    Eigen::MatrixXd result(q_traj.rows(), 22);
    result << time, q_traj;

    std::string path_out = "../../out/joint_angles.csv";
    const static Eigen::IOFormat CSVFormat(Eigen::FullPrecision, Eigen::DontAlignCols, ", ", ",\n");
    std::ofstream file(path_out.c_str());
    file << "COLUMNS:\n"
         << "time,\n"
         << "root_link:T:X,\n"
         << "root_link:T:Y,\n"
         << "root_link:T:Z,\n"
         << "root_link:R:X:rad,\n"
         << "root_link:R:Y:rad,\n"
         << "root_link:R:Z:rad,\n"
         << "l_hip_1:R:X:rad,\n"
         << "l_hip_2:R:-Z:rad,\n"
         << "l_upper_leg:R:Y:rad,\n"
         << "l_lower_leg:R:X:rad,\n"
         << "l_ankle_1:R:-X:rad,\n"
         << "l_ankle_2:R:-Z:rad,\n"
         << "r_hip_1:R:-X:rad,\n"
         << "r_hip_2:R:-Z:rad,\n"
         << "r_upper_leg:R:-Y:rad,\n"
         << "r_lower_leg:R:-X:rad,\n"
         << "r_ankle_1:R:X:rad,\n"
         << "r_ankle_2:R:-Z:rad\n"
         << "torso_1:R:X:rad,\n"
         << "torso_2:R:-Z:rad,\n"
         << "chest:R:-Y:rad\n"
         << "DATA:\n"
         << result.format(CSVFormat);
*/

    // Save raw results.

    // Save interpolated results.
    Eigen::MatrixXd trajectories = interpol_nmpc.GetTrajectories().transpose();
    WriteCsv("example_nmpc_generator_interpolated_results.csv", trajectories);
}

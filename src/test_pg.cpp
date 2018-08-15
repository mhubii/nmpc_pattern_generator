#include <iostream>

#include "nmpc_generator.h"
#include "interpolation.h"

int main() {
    
    // Initialize pattern generator.
    const std::string config_file_loc = "../libs/pattern_generator/configs.yaml";

    NMPCGenerator nmpc(config_file_loc);

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
    Interpolation ip(nmpc);
    ip.StoreTrajectories(true);
    Eigen::Vector3d velocity_reference(0.2, 0., 0.);


    Eigen::MatrixXd traj = ip.GetTrajectoriesBuffer();;
    Eigen::MatrixXd com_traj(4, 1);
    Eigen::MatrixXd lf_traj(4, 1);
    Eigen::MatrixXd rf_traj(4, 1);

    int intervals = 400;

    // TEST
    Eigen::MatrixXd trajectories(21, intervals);
    // TEST END


    // Pattern generator event loop.
    for (int i = 0; i < intervals; i++) {
        
        std::cout << "Iteration: " << i << std::endl;

        // TEST
        // Provide feedback. Later replaced by forward kinematics.
        pg_state.com_x(0) = traj(0, 0);
        pg_state.com_y(0) = traj(3, 0);
        pg_state.com_z = traj(6, 0);

        nmpc.SetInitialValues(pg_state);
        // TEST END


        // Set reference velocities.
        nmpc.SetVelocityReference(velocity_reference);


        // Solve QP.
        nmpc.Solve();
        nmpc.Simulate();
        // ip.InterpolateStep();

        // pg_state = nmpc.Update();
        // nmpc.SetInitialValues(pg_state);

        // TEST
        // ip.SetInitialValues(lf_traj, rf_traj);


        // Initial value embedding by internal states and simulation.
        if (ip.GetCurrentInterval() % ip.GetIntervals() == 0) {

            pg_state = nmpc.Update();
        }
        // TEST END

        // TEST
        traj = ip.Interpolate();
        trajectories.col(i) = traj;

        com_traj << traj(0, 1),  traj(3, 1),  traj(6, 1),  traj(7, 1);
        lf_traj  << traj(13, 1), traj(14, 1), traj(15, 1), traj(16, 1);
        rf_traj  << traj(17, 1), traj(18, 1), traj(19, 1), traj(20, 1);
        // TEST END
    }


    // Save interpolated results.
    Eigen::MatrixXd stored = ip.GetTrajectories();
    WriteCsv("example_nmpc_generator_interpolated_results.csv", stored.transpose());
    // // TEST
    // WriteCsv("example_nmpc_generator_interpolated_results.csv", trajectories.transpose());
    // // TEST END
}
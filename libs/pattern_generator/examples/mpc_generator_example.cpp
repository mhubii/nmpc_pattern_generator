#include <iostream>

#include "mpc_generator.h"
#include "interpolation.h"
#include "utils.h"
#include "timer.h"

int main() {
    // Instantiate pattern generator.
    const std::string config_file_loc = "../../libs/pattern_generator/configs.yaml";

    MPCGenerator mpc(config_file_loc);

    // Pattern generator preparation.
    mpc.SetSecurityMargin(mpc.SecurityMarginX(), 
                          mpc.SecurityMarginY());

    // Set initial values.
    PatternGeneratorState pg_state = {mpc.Ckx0(),
                                      mpc.Cky0(),
                                      mpc.Hcom(),
                                      mpc.Fkx0(),
                                      mpc.Fky0(),
                                      mpc.Fkq0(),
                                      mpc.CurrentSupport().foot,
                                      mpc.Ckq0()};

    mpc.SetInitialValues(pg_state);
    Interpolation interpol_mpc(mpc);
    interpol_mpc.StoreTrajectories(true);
    Eigen::Vector3d velocity_reference(0.1, 0., 0.1);

    Timer(START);

    // Pattern generator event loop.
    for (int i = 0; i < 200; i++) {
        std::cout << "Iteration: " << i << std::endl;

        // Change reference velocities.
        if (25 <= i && i < 50) {
        velocity_reference << 0.1, 0., 0.1;
        }
        else if (50 <= i && i < 150) {
        velocity_reference << 0.1, 0.1, 0.1;
        }
        else if (150 <= i && i < 200) {
        velocity_reference << 0., 0., 0.;
        }


        // Set reference velocities.
        mpc.SetVelocityReference(velocity_reference);

    
        // Solve QP.
        mpc.Solve();
        mpc.Simulate();
        interpol_mpc.InterpolateStep();

        
        // Initial values embedding by internal states and simulation.
        pg_state = mpc.Update();
        mpc.SetInitialValues(pg_state);
    }

    double elapsed_time = Timer(STOP);
    std::cout << "Elapsed time: " << elapsed_time << " ms" << std::endl;

    // Save interpolated results.
    Eigen::MatrixXd trajectories = interpol_mpc.GetTrajectories().transpose();
    WriteCsv("example_mpc_generator_interpolated_results.csv", trajectories);
}
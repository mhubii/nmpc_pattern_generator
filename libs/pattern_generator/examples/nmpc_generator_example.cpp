#include <iostream>

#include "nmpc_generator.h"
#include "interpolation.h"

int main() {
    // Initialize pattern generator.
    const std::string config_file_loc = "../../libs/pattern_generator/configs.yaml";

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
    Interpolation interpol_nmpc(nmpc);
    interpol_nmpc.StoreTrajectories(true);
    Eigen::Vector3d velocity_reference(0., 0., 0.1);


    // Pattern generator event loop.
    for (int i = 0; i < 400; i++) {
        std::cout << "Iteration: " << i << std::endl;


        // Change reference velocities.
        if (50 <= i && i < 100) {
            velocity_reference << 0.1, 0., 0.1;
        }
        else if (100 <= i && i < 300) {
            velocity_reference << 0.1, 0., -0.1;
        }
        else if (300 <= i && i < 400) {
            velocity_reference << 0., 0., 0.;
        }


        // Set reference velocities.
        nmpc.SetVelocityReference(velocity_reference);


        // Solve QP.
        nmpc.Solve();
        nmpc.Simulate();
        interpol_nmpc.InterpolateStep();


        // Initial value embedding by internal states and simulation.
        pg_state = nmpc.Update();
        nmpc.SetInitialValues(pg_state);
    }


    // Save interpolated results.
    Eigen::MatrixXd trajectories = interpol_nmpc.GetTrajectories().transpose();
    WriteCsv("example_nmpc_generator_interpolated_results.csv", trajectories);
}
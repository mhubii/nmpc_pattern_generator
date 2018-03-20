#include <iostream>

#include "nmpc_generator.h"
#include "interpolation.h"

int main() {
    // Initialize pattern generator.
    const int n = 16;
    const double t = 0.1;
    const double t_step = 0.8;
    const std::string fsm_state = "L/R";

    NMPCGenerator nmpc(n, t, t_step, fsm_state);


    // Pattern generator preparation.
    nmpc.SetSecurityMargin(0.02, 0.02);


    // Set initial values.
    Eigen::Vector3d com_x(0., 0., 0.);
    Eigen::Vector3d com_y(0.04, 0., 0.);
    double com_z = 0.46;
    double foot_x = 0.;
    double foot_y = 0.07;
    double foot_q = 0.;
    std::string foot = "left";
    Eigen::Vector3d com_q(0., 0., 0.);

    PatternGeneratorState pg_state = {com_x,
                                    com_y,
                                    com_z,
                                    foot_x,
                                    foot_y,
                                    foot_q,
                                    foot,
                                    com_q};

    nmpc.SetInitialValues(pg_state);
    Interpolation interpol_nmpc(0.005, nmpc);
    Eigen::Vector3d velocity_reference(0.1, 0., 0.);


    // Pattern generator event loop.
    for (int i = 0; i < 200; i++) {
        std::cout << "Iteration: " << i << std::endl;
        double time = i*0.1;


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
        nmpc.SetVelocityReference(velocity_reference);


        // Solve QP.
        nmpc.Solve();
        nmpc.Simulate();
        interpol_nmpc.Interpolate(time);


        // Initial value embedding by internal states and simulation.
        pg_state = nmpc.Update();
        nmpc.SetInitialValues(pg_state);
    }


    // Save interpolated results.
    interpol_nmpc.SaveToFile("example_nmpc_generator_interpolated_results.csv");
}
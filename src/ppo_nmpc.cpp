#include "nmpc_generator.h"
#include "interpolation.h"
#include "proximal_policy_optimization.h"
#include "timer.h"

// // Pattern generator event loop.
// for (int i = 0; i < 400; i++) {
//     std::cout << "Iteration: " << i << std::endl;


//     // Change reference velocities.
//     if (50 <= i && i < 100) {
//         velocity_reference << 0.1, 0., 0.1;
//     }
//     else if (100 <= i && i < 300) {
//         velocity_reference << 0.1, 0., -0.1;
//     }
//     else if (300 <= i && i < 400) {
//         velocity_reference << 0., 0., 0.;
//     }


//     // Set reference velocities.
//     nmpc.SetVelocityReference(velocity_reference);


//     // Solve QP.
//     nmpc.Solve();
//     nmpc.Simulate();
//     interpol_nmpc.InterpolateStep();


//     // Initial value embedding by internal states and simulation.
//     pg_state = nmpc.Update();
//     nmpc.SetInitialValues(pg_state);
// }


// // Save interpolated results.
// Eigen::MatrixXd trajectories = interpol_nmpc.GetTrajectories().transpose();
// WriteCsv("example_nmpc_generator_interpolated_results.csv", trajectories);



// just do it
// goal of this example will be to use a combination of nmpc and ppo to solve navigation
// for a humanoid robot
//   - setup nmpc first and to try and spawn random obstacles
//     in a map
//   - then, use ppo to reach a goal on two different setups
//     - provide the nmpc with information about the obstacle
//     - dont provide, then measure the time it takes to converge, given a certain metric

int main(int argc, char** argv)
{
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
    Eigen::Vector3d velocity_reference(0.01, 0., 0.);

    nmpc.Reset();

    return 0;
}
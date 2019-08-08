#include <iostream>
#include <fstream>

#include "nmpc_generator.h"
#include "interpolation.h"
#include "utils.h"
#include "timer.h"

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
    Eigen::Vector3d velocity_reference(0.01, 0., 0.);

    Timer(START);

    // Pattern generator event loop.
    for (int i = 0; i < 1000; i++) {
        std::cout << "Iteration: " << i << std::endl;

        // // Change reference velocities.
        // if (25 <= i && i < 50) {
        //     velocity_reference << 0.1, 0., 0.05;
        // }
        // else if (50 <= i && i < 175) {
        //     velocity_reference << 0.1, 0., 0.05;
        // }
        // else if (175 <= i && i < 200) {
        //     velocity_reference << 0., 0., 0.;
        // }

        // Set reference velocities.
        nmpc.SetVelocityReference(velocity_reference);


        // Solve QP.
        nmpc.Solve();
        nmpc.Simulate();
        auto inter = interpol_nmpc.InterpolateStep();
        
        auto preview = int(interpol_nmpc.preview_intervals_);
        auto dt = nmpc.CpuTime();
        auto com_x = inter.block(0, preview, 3, 1);// initial values not final
        auto com_y = inter.block(3, preview, 3, 1);
        auto lfx = inter(13, preview);
        auto lfy = inter(14, preview);
        auto rfx = inter(17, preview);
        auto rfy = inter(18, preview);

        // Initial value embedding by internal states and simulation.
        pg_state = nmpc.Update(dt);
        pg_state.com_x = com_x;
        pg_state.com_y = com_y;
        if (nmpc.current_support_.foot == "left") {
            pg_state.foot_x = lfx;
            pg_state.foot_y = lfy;
        }
        else {
            pg_state.foot_x = rfx;
            pg_state.foot_y = rfy;
        }
        nmpc.SetInitialValues(pg_state);
    }

    double elapsed_time = Timer(STOP);
    std::cout << "Elapsed time: " << elapsed_time << " ms" << std::endl;

    // Save interpolated results.
    Eigen::MatrixXd trajectories = interpol_nmpc.GetTrajectories().transpose();
    WriteCsv("example_nmpc_generator_interpolated_results.csv", trajectories);
}
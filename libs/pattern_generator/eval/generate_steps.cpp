#include <string.h>

#include "nmpc_generator.h"
#include "kinematics.h"

std::string config_file_loc;
uint iter;
Eigen::Vector3d vel;

void ReadCmd(int argc, char** argv) {

    for (int i = 0; i < argc - 1; i++) {

        if (!std::strcmp(argv[i], "-l")) {

            config_file_loc = argv[i+1];
        }
        else if (!std::strcmp(argv[i], "-i")) {

            iter = atoi(argv[i+1]);
        }
        else if (!std::strcmp(argv[i], "-v")) {
            
            vel(0) = atof(argv[i + 1]);
            vel(1) = atof(argv[i + 2]); 
            vel(2) = atof(argv[i + 3]);
        }
        
    }

    if (config_file_loc.empty()) {

        std::cerr << "Please provide a config file with the flag -l <filelocation>, the number of iterations with the flag -i <iterations>, and the desied velocity with the flag -v <vx vy omegaz>" << std::endl;
        std::exit(1);
    }
}


int main(int argc, char** argv) {

    // Read from the command line.
    ReadCmd(argc, argv);

    // Initialize pattern generator.
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

    // Record the states.
    Eigen::MatrixXd states(13, 0);
    Eigen::MatrixXd states_preview(7, 0);

    // Pattern generator event loop.
    for (int i = 0; i < iter; i++) {
        std::cout << "Iteration: " << i << std::endl;

        // Set reference velocities.
        nmpc.SetVelocityReference(vel);


        // Solve QP.
        nmpc.Solve();
        nmpc.Simulate();
        // interpol_nmpc.InterpolateStep();


        // Initial value embedding by internal states and simulation.
        pg_state = nmpc.Update();
        nmpc.SetInitialValues(pg_state);

        // Record the current states.
        states.conservativeResize(states.rows(), states.cols() + 1);
        states_preview.conservativeResize(states_preview.rows(), states_preview.cols() + nmpc.N());

        // Current com.
        states.middleRows(0, 3).rightCols(1) = nmpc.Ckx0();
        states.middleRows(3, 3).rightCols(1) = nmpc.Cky0();
        states.middleRows(6, 3).rightCols(1) = nmpc.Ckq0();
        states(9, states.cols() - 1) = nmpc.Hcom();

        // Preview com.
        states_preview.row(0).rightCols(nmpc.N()) = nmpc.Ckp1x().transpose();
        states_preview.row(1).rightCols(nmpc.N()) = nmpc.Ckp1y().transpose();
        states_preview.row(2).rightCols(nmpc.N()) = nmpc.Ckp1q().transpose();
        states_preview.row(3).rightCols(nmpc.N()) = Eigen::MatrixXd::Constant(1, nmpc.N(), nmpc.Hcom());

        // Current feet.
        states(10, states.cols() - 1)  = nmpc.Fkx0();
        states(11, states.cols() - 1) = nmpc.Fky0();
        states(12, states.cols() - 1) = nmpc.Fkq0();

        // Preview feet.
        states_preview.row(4).rightCols(nmpc.N()) = Eigen::MatrixXd::Constant(1, nmpc.N(), nmpc.Fkx0());
        states_preview.row(5).rightCols(nmpc.N()) = Eigen::MatrixXd::Constant(1, nmpc.N(), nmpc.Fky0());
        states_preview.row(6).rightCols(nmpc.N()) = Eigen::MatrixXd::Constant(1, nmpc.N(), nmpc.Fkq0());
    }


    // Save interpolated results.
    Eigen::MatrixXd trajectories = interpol_nmpc.GetTrajectories().transpose();
    WriteCsv("states.csv", states);
    WriteCsv("states_preview.csv", states_preview);

    return 0;
}
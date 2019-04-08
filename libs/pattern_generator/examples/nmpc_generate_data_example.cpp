#include <iostream>
#include <qpOASES.hpp>

#include "nmpc_generator.h"
#include "interpolation.h"
#include "utils.h"
#include "timer.h"

int main() {

    // Random velocity generation.
    std::random_device rd;
    std::mt19937 re(rd());
    // std::uniform_real_distribution<double> dist(0., 1.);
    int range = 10;
    std::uniform_int_distribution<> dist(1, range);

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
    Eigen::Vector3d velocity_reference(0., 0., 0.);

    int epochs = 1e3;
    int iter = 2e1;

    // Store trajectories. Optimal control := oc, preview horizon := ph.
    Eigen::MatrixXd init = Eigen::MatrixXd::Zero(iter, nmpc.LocalVelRef().size()+
                                                       nmpc.Ckx0().size()+
                                                       nmpc.Cky0().size()+
                                                       nmpc.Ckq0().size()+
                                                       2 /*fkx, fky*/);
                                                                                
    Eigen::MatrixXd oc_ph_com = Eigen::MatrixXd::Zero(iter, nmpc.Dddckx().size()+
                                                            nmpc.Dddcky().size());

    Eigen::MatrixXd oc_ph_fkq = Eigen::MatrixXd::Zero(iter, nmpc.Dddfkql().size()+
                                                            nmpc.Dddfkqr().size());

    Eigen::MatrixXd oc_ph_fkp = Eigen::MatrixXd::Zero(iter, nmpc.Fkx().size()+
                                                            nmpc.Fky().size());

    std::ofstream out;
    out.open("data/successful_epochs.csv");

    bool save = true; // dont save results on unsuccessful solution

    for (int e = 0; e < epochs; e++) {
        printf("Epoch %d/%d\n", e, epochs);

        // Generate random velocity.
        velocity_reference <<  double(dist(re))*0.1/(double)range, 0., 0.;// dist(re)/5., dist(re)/10.;
        std::cout << velocity_reference.transpose() << std::endl;
        // velocity_reference << 0.01, 0., 0.;

        // Pattern generator event loop.
        for (int i = 0; i < iter; i++) {

            // Set reference velocities.
            nmpc.SetVelocityReference(velocity_reference);

            // Save the initial state.
            init.row(i) << nmpc.LocalVelRef().transpose(), 
                           nmpc.Ckx0().transpose(), 
                           nmpc.Cky0().transpose(), 
                           nmpc.Ckq0().transpose(),
                           nmpc.Fkx0(), nmpc.Fky0(); 

            // Solve QP.
            nmpc.Solve();

            if (nmpc.GetStatus() != qpOASES::SUCCESSFUL_RETURN) {
                save = false;
                break;
            }

            // Save the solutions of the OCP.
            oc_ph_com.row(i) << nmpc.Dddckx().transpose(), 
                                nmpc.Dddcky().transpose();

            oc_ph_fkq.row(i) << nmpc.Dddfkql().transpose(), 
                                nmpc.Dddfkqr().transpose();

            oc_ph_fkp.row(i) << nmpc.Fkx().transpose(), 
                                nmpc.Fky().transpose();

            // Simulate the preview horizon, given the current state and the jerks.
            nmpc.Simulate();

            // Initial value embedding by internal states and simulation.
            pg_state = nmpc.Update();
            nmpc.SetInitialValues(pg_state);
        }
    
        // Save results.
        if (save) {
            out << e+1 << "\n";
            WriteCsv("data/ini_epoch_" + std::to_string(e+1) + ".csv", init);
            WriteCsv("data/com_epoch_" + std::to_string(e+1) + ".csv", oc_ph_com);
            WriteCsv("data/fkq_epoch_" + std::to_string(e+1) + ".csv", oc_ph_fkq);
            WriteCsv("data/fkp_epoch_" + std::to_string(e+1) + ".csv", oc_ph_fkp);
        }

        save = true;

        init.setZero();
        oc_ph_com.setZero();
        oc_ph_fkq.setZero();
        oc_ph_fkp.setZero();

        // Reset the pattern generator.
        nmpc.Reset();

        pg_state = {nmpc.Ckx0(),
                    nmpc.Cky0(),
                    nmpc.Hcom(),
                    nmpc.Fkx0(),
                    nmpc.Fky0(),
                    nmpc.Fkq0(),
                    nmpc.CurrentSupport().foot,
                    nmpc.Ckq0()};

        nmpc.SetInitialValues(pg_state);
    }

    out.close();
}
#include <Eigen/Core>
#include <torch/torch.h>
#include <chrono>

#include "base_generator.h"
#include "nmpc_generator.h"
#include "interpolation.h"
#include "utils.h"
#include "models.h"
#include "timer.h"

using namespace std::chrono;

Eigen::MatrixXd ori_sol = Eigen::MatrixXd::Zero(32,1);

// Generate and save reference using the classical nmpc.
void Nmpc(NMPCGenerator& nmpc) 
{
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
    Eigen::Vector3d velocity_reference(0.01, 0., 0.);


    // Set reference velocities.
    nmpc.SetVelocityReference(velocity_reference);


    // Solve QP.
    auto t1 = high_resolution_clock::now();
    nmpc.Solve();
    auto t2 = high_resolution_clock::now();
    printf("Nmpc elapsed time: %f ms\n", (double)duration_cast<nanoseconds>(t2 - t1).count()/1000.);
    nmpc.Simulate();


    // Initial value embedding by internal states and simulation.
    pg_state = nmpc.Update();
    nmpc.SetInitialValues(pg_state);

    // Save jerks.
    Eigen::MatrixXd com_jerks = Eigen::MatrixXd::Zero(nmpc.N(), 2);
    com_jerks << nmpc.Dddckx(), nmpc.Dddcky();
    WriteCsv("nmpc_jerks.csv", com_jerks);
    ori_sol << nmpc.Dddckx(), nmpc.Dddcky();

    // Save results.
    Eigen::MatrixXd com_xy_preview = Eigen::MatrixXd::Zero(nmpc.N(), 2);
    com_xy_preview << nmpc.Ckp1x(), nmpc.Ckp1y();
    WriteCsv("nmpc_com_preview_horizon.csv", com_xy_preview);
}


// Simulate the time evolution of the system, given the solution of the optimal control and the initial state.
auto Simulate(torch::Tensor& pps,
              torch::Tensor& ppu,
              torch::Tensor& init,
              torch::Tensor& solution) -> std::tuple<torch::Tensor, torch::Tensor>
{
    int n = pps.size(0);

    // Initial state.
    auto c_k_x_0 = init.slice(1, 3, 6).transpose(0,1);
    auto c_k_y_0 = init.slice(1, 6, 9).transpose(0,1);

    // Jerk on preview horizon.
    auto dddc_k_x = solution.slice(1, 0, n).transpose(0,1); 
    auto dddc_k_y = solution.slice(1, n, 2*n).transpose(0,1);

    // Position on preview horizon.
    auto c_kp1_x = pps.matmul(c_k_x_0) + ppu.matmul(dddc_k_x);
    auto c_kp1_y = pps.matmul(c_k_y_0) + ppu.matmul(dddc_k_y);

    return std::make_tuple(c_kp1_x, c_kp1_y);
}

// Generate and save patterns using the trained neural net.
void Net(NmpcNet& net, BaseGenerator base)
{
    // Get the predictive matrices and convert them to tensors.
    RowMatrixXd pps = base.PPS();
    RowMatrixXd ppu = base.PPU();

    torch::Tensor t_pps = torch::zeros({pps.rows(), pps.cols()}, torch::kF64);
    torch::Tensor t_ppu = torch::zeros({ppu.rows(), ppu.cols()}, torch::kF64);

    std::memcpy(t_pps.data_ptr(), pps.data(), pps.size()*sizeof(double));
    std::memcpy(t_ppu.data_ptr(), ppu.data(), ppu.size()*sizeof(double));

    // Get the initial state.
    RowMatrixXd init = Eigen::MatrixXd::Zero(1, base.LocalVelRef().size()+
                                                base.Ckx0().size()+
                                                base.Cky0().size()+
                                                base.Ckq0().size()+
                                                2 /*fkx, fky*/);

    Eigen::Vector3d velocity_reference(0.01, 0., 0.);

    init << velocity_reference.transpose(),
            base.Ckx0().transpose(), 
            base.Cky0().transpose(), 
            base.Ckq0().transpose(),
            base.Fkx0(), base.Fky0(); 

    torch::Tensor t_init = torch::zeros({init.rows(), init.cols()}, torch::kF64);

    std::memcpy(t_init.data_ptr(), init.data(), init.size()*sizeof(double));

    // Emulate the nonlinear predictive control with the trained neural net.  
    auto t1 = high_resolution_clock::now();
    torch::Tensor solution = net->forward(t_init);
    auto t2 = high_resolution_clock::now();
    printf("Net elapsed time: %f ns\n", (double)duration_cast<nanoseconds>(t2 - t1).count()/1000.);

    // Jerk on preview horizon.
    int n = base.N();
    auto t_dddc_k_x = solution.slice(1, 0, n); 
    auto t_dddc_k_y = solution.slice(1, n, 2*n);

    RowMatrixXd dddc_k_x = Eigen::MatrixXd::Zero(1, n);
    RowMatrixXd dddc_k_y = Eigen::MatrixXd::Zero(1, n);

    std::memcpy(dddc_k_x.data(), t_dddc_k_x.data_ptr(), dddc_k_x.size()*sizeof(double));
    std::memcpy(dddc_k_y.data(), t_dddc_k_y.data_ptr(), dddc_k_y.size()*sizeof(double));

    // Save jerks.
    Eigen::MatrixXd com_jerks = Eigen::MatrixXd::Zero(n, 2);
    com_jerks << dddc_k_x.transpose(), dddc_k_y.transpose();
    WriteCsv("net_jerks.csv", com_jerks);

    auto com = Simulate(t_pps, t_ppu, t_init, solution);

    // Convert to matrix again.
    RowMatrixXd ckp1x = Eigen::MatrixXd::Zero(base.N(), 1);
    RowMatrixXd ckp1y = Eigen::MatrixXd::Zero(base.N(), 1);

    std::memcpy(ckp1x.data(), std::get<0>(com).data_ptr(), ckp1x.size()*sizeof(double));
    std::memcpy(ckp1y.data(), std::get<1>(com).data_ptr(), ckp1y.size()*sizeof(double));

    // Save results.
    Eigen::MatrixXd com_xy_preview = Eigen::MatrixXd::Zero(base.N(), 2);
    com_xy_preview << ckp1x, ckp1y;
    WriteCsv("net_com_preview_horizon.csv", com_xy_preview);
}


int main()
{
    // Initialize pattern generator.
    const std::string config_file_loc = "/home/martin/Downloads/nmpc_pattern_generator/libs/pattern_generator/configs.yaml";

    NMPCGenerator nmpc(config_file_loc);

    // Run the nonlinear model predictive control.
    Nmpc(nmpc);
    nmpc.Reset();

    // Load the trained net.
    int64_t n_in = 14;
    int64_t n_out = 32;

    NmpcNet net(n_in, n_out);

    printf("Loading the network...\n");
    torch::load(net, "/home/martin/Downloads/nmpc_pattern_generator/build/bin/best_model.pt");
    printf("Finished loading the network.\n");

    net->to(torch::kCPU);
    net->to(torch::kF64);
    net->eval();

    // Run the trained net to perform nonlinear model prdictive control.
    Net(net, nmpc);
}
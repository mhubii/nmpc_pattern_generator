#include <Eigen/Core>
#include <qpOASES.hpp>
#include <random>
#include <cmath>
#include <torch/torch.h>

#include "nmpc_generator.h"
#include "interpolation.h"
#include "utils.h"
#include "proximal_policy_optimization.h"
#include "timer.h"

std::random_device rdev;  //Will be used to obtain a seed for the random number engine
std::mt19937 gen(rdev()); //Standard mersenne_twister_engine seeded with rd()
std::uniform_real_distribution<> random_real(-4., 4.); // generate a random boolean

enum STATUS {
    PLAYING,
    REACHEDGOAL,
    MISSEDGOAL,
    HITOBSTACLE,
    RESETTING,
    ERROR
};

// Environment for nmpc to navigate in. The state of the environment is 
// represented by the position of the goal and the com as well as 
// a matrix representing the surroundings.
struct NMPCEnvironment
{
    NMPCGenerator nmpc_;
    Interpolation interpol_nmpc_;
    PatternGeneratorState pg_state_;

    Eigen::Vector2d pos_;
    Eigen::Vector3d vel_;
    Eigen::Vector2d obs_;
    double r_obs_;
    Eigen::Vector2d goal_;
    Eigen::VectorXd state_;

    double old_dist_;
    double old_preview_dist_;


    // Constructor.
    NMPCEnvironment(std::string config_file_loc, double x_goal, double y_goal) : nmpc_(config_file_loc), interpol_nmpc_(nmpc_), state_(2)
    {
        // Pattern generator preparation.
        nmpc_.SetSecurityMargin(nmpc_.SecurityMarginX(), 
                                nmpc_.SecurityMarginY());

        pg_state_ = {nmpc_.Ckx0(),
                     nmpc_.Cky0(),
                     nmpc_.Hcom(),
                     nmpc_.Fkx0(),
                     nmpc_.Fky0(),
                     nmpc_.Fkq0(),
                     nmpc_.CurrentSupport().foot,
                     nmpc_.Ckq0()};

        nmpc_.SetInitialValues(pg_state_);

        // Update state.
        pos_ << nmpc_.Ckx0()[0], nmpc_.Cky0()[0];
        vel_ << nmpc_.LocalVelRef();
        obs_ << nmpc_.XObs(), nmpc_.YObs();
        r_obs_ = nmpc_.RObs();
        
        // Set new goal.
        // double x_new = double(random_real(gen)); 
        // double y_new = double(random_real(gen));
        SetGoal(x_goal, y_goal);

        auto theta = nmpc_.Ckq0()(0);
        Eigen::Matrix2d rot;
        rot <<   cos(theta), sin(theta),
                -sin(theta), cos(theta);
        auto goal_trafo = rot*(goal_ - pos_); 

        state_ << goal_trafo;

        old_dist_ = (goal_ -  pos_).norm();
        old_preview_dist_ = PreviewDist();
    };

    auto State() -> torch::Tensor
    {
        // Return position and goal.
        torch::Tensor state = torch::zeros({1, state_.size()}, torch::kF64);
        std::memcpy(state.data_ptr(), state_.data(), state_.size()*sizeof(double));

        return state;
    }

    // Functions to interact with the environment for reinforcement learning.
    auto Act(Eigen::Vector3d vel) -> std::tuple<torch::Tensor /*position*/, int, torch::Tensor>
    {
        old_dist_ = (goal_ -  pos_).norm();
        old_preview_dist_ = PreviewDist();

        // Run nmpc for one episode.
        vel_ = vel;

        nmpc_.SetVelocityReference(vel_);
        nmpc_.Solve();
        nmpc_.Simulate();
        interpol_nmpc_.InterpolateStep();

        pg_state_ = nmpc_.Update();
        nmpc_.SetInitialValues(pg_state_);

        // Update state.
        pos_ << nmpc_.Ckx0()[0], nmpc_.Cky0()[0];
        vel_ << nmpc_.LocalVelRef();

        auto theta = nmpc_.Ckq0()(0);
        Eigen::Matrix2d rot;
        rot <<   cos(theta), sin(theta),
                -sin(theta), cos(theta);
        auto goal_trafo = rot*(goal_ - pos_); 

        state_ << goal_trafo;

        torch::Tensor done = torch::zeros({1, 1}, torch::kF64);
        STATUS status;

        if ((goal_ - pos_).norm() < 4e-1) {
            status = REACHEDGOAL;
            done[0][0] = 1.;
        }
        else if ((goal_ - pos_).norm() > 40.0) {
            status = MISSEDGOAL;
            done[0][0] = 1.;
        }
        else {
            status = PLAYING;
            done[0][0] = 0.;
        }
        if (nmpc_.GetStatus() != qpOASES::SUCCESSFUL_RETURN) {
            status = STATUS::ERROR;
            done[0][0] = 0.;
        }

        auto state = State();

        return std::make_tuple(state, status, done);
    };

    auto Reward(int status) -> torch::Tensor
    {
        double goal_factor = 1e2;
        auto dist = (goal_ - pos_).norm();

        auto theta = nmpc_.Ckq0()(0);
        // std::cout << "ckx0: " << nmpc_.Ckx0().transpose() << std::endl;
        // std::cout << "ckq0: " << nmpc_.Ckp1q().transpose() << std::endl;
        // std::cout << "theta: " << theta << std::endl;
        Eigen::Matrix2d rot;
        rot <<   cos(theta), sin(theta),
                -sin(theta), cos(theta);
        // std::cout << "rot: \n" << rot << std::endl;
        auto goal_trafo = rot*(goal_ - pos_); 
        // std::cout << "goal_trafo: " << goal_trafo.transpose() << std::endl;
        auto ori = goal_trafo.normalized();

        // torch::Tensor reward = torch::full({1, 1}, goal_factor*(old_dist_ - dist), torch::kF64);
        std::cout << "dir: " << ori(0) << " dist: " << 2e2*(old_dist_-dist) << std::endl;
        torch::Tensor reward = torch::full({1, 1}, 2e2*goal_factor*(old_dist_-dist), torch::kF64);

        switch (status)
            {
                case PLAYING:
                    break;
                case REACHEDGOAL:
                    reward[0][0] += 1e2;
                    printf("reached goal, reward: %f\n", *(reward.cpu().data<double>()));
                    break;
                case MISSEDGOAL:
                    reward[0][0] -= 1e2;
                    printf("missed goal, reward: %f\n", *(reward.cpu().data<double>()));
                    break;
                case HITOBSTACLE:
                    reward[0][0] -= 1e2;
                    printf("hit obstacle, reward: %f\n", *(reward.cpu().data<double>()));
                    break;
            }

        return reward;
    };

    auto PreviewDist() -> double
    {
        // Compute the average distance of the preview horizon from the goal.
        auto x_dist = (nmpc_.Ckp1x().array() - goal_(0));
        auto y_dist = (nmpc_.Ckp1y().array() - goal_(1));

        auto x_dist_sq = x_dist*x_dist;
        auto y_dist_sq = y_dist*y_dist;

        auto dist = (x_dist_sq + y_dist_sq).sqrt();

        return dist.matrix().sum()/dist.size();
    }

    auto PreviewAvgPos() -> Eigen::Vector2d
    {
        // Returns the averaged position on the preview horizon.
        auto x = nmpc_.Ckp1x().mean();
        auto y = nmpc_.Ckp1y().mean();

        return Eigen::Vector2d(x, y);
    }

    auto Reset() -> void
    {
        nmpc_.Reset();

        pg_state_ = {nmpc_.Ckx0(),
                     nmpc_.Cky0(),
                     nmpc_.Hcom(),
                     nmpc_.Fkx0(),
                     nmpc_.Fky0(),
                     nmpc_.Fkq0(),
                     nmpc_.CurrentSupport().foot,
                     nmpc_.Ckq0()};

        nmpc_.SetInitialValues(pg_state_);

        // Reset the states.
        pos_ << nmpc_.Ckx0()[0], nmpc_.Cky0()[0];
        vel_ << nmpc_.LocalVelRef();

        auto theta = nmpc_.Ckq0()(0);
        Eigen::Matrix2d rot;
        rot <<   cos(theta), sin(theta),
                -sin(theta), cos(theta);
        auto goal_trafo = rot*(goal_ - pos_); 

        state_ << goal_trafo;

        // Set new goal.
        // double x_new = double(random_real(gen)); 
        // double y_new = double(random_real(gen));
        // SetGoal(x_new, y_new);
    };

    auto SetGoal(double x, double y) -> void
    {
        goal_(0) = x;
        goal_(1) = y;

        auto theta = nmpc_.Ckq0()(0);
        Eigen::Matrix2d rot;
        rot <<   cos(theta), sin(theta),
                -sin(theta), cos(theta);
        auto goal_trafo = rot*(goal_ - pos_); 

        old_dist_ = (goal_ -  pos_).norm();
        old_preview_dist_ = PreviewDist();

        state_ << goal_trafo;
    };

    auto SetObstacle() -> void
    {
	    // // Create central path.
        // int roi_x_size = int(nx_*2./3.);
	    // int roi_y_size = int(ny_/2.);
        // int roi_x_pos = (nx_ - roi_x_size)/2.;
        // int roi_y_pos = (ny_ - roi_y_size)/2.;

        // roi_x_pos += int(roi_x_size/6.); // compute the center of the gaussian distribution in image coordinates
        // roi_y_pos += int(roi_y_size/2.);

        // obs_(0) = (roi_x_pos - x_off_)*dx_;
        // obs_(1) = (roi_y_pos - y_off_)*dy_ + 0.1;

        // r_obs_ = double(roi_y_size)/60.*dx_; // same size as sigmax (maybe a little smaller)  //double(roi_y_size)/4.*dx_; // roughly size of the roi/4
        // double r_margin = nmpc_.RMargin();

        // Circle c{obs_(0), obs_(1), r_obs_, r_margin};

        // nmpc_.SetObstacle(c);

        // // Update state.
        // state_ << pos_, goal_;
    };
};

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
    // Setup the nonlinear model predictive control.
    std::string config_file_loc = argv[1];
    double x_goal = std::stod(argv[2]);
    double y_goal = std::stod(argv[3]);
    NMPCEnvironment env(config_file_loc, x_goal, y_goal);

    env.Reset();

    // Proximal policy optimization. We use informations of the states system as well as its environment for n_in.
    torch::DeviceType device_type;
    // if (torch::cuda::is_available()) {
    //     printf("CUDA available! Training on GPU.\n");
    //     device_type = torch::kCUDA;
    // } 
    // else {
    //     printf("Training on CPU.\n");
        device_type = torch::kCPU;
    // }
    torch::Device device(device_type);

    uint n_in = env.state_.size();
    uint n_out = 2;//env.vel_.size();
    double mu_max = 0.1;
    double std = 5e-2;

    ActorCriticNMPC ac(n_in, n_out, mu_max, std);
    ac->to(torch::kF64);
    ac->normal(0., 1e-2);
    ac->to(device);
    ac->eval();
    torch::load(ac, "../../out/ppo_nmpc/ppo_nmpc_best_model.pt");

    // Training loop.
    uint n_iter = 4000;
    uint n_steps = 2000;
    uint n_epochs = 10;
    uint mini_batch_size = 500;
    uint ppo_epochs = 4;
    double beta = 1e-2;

    // Output.
    std::ofstream out;
    out.open("../../out/ppo_nmpc/test_example_ppo.csv");

    // episode, agent_x, agent_y, goal_x, goal_y, STATUS=(PLAYING, REACHEDGOAL, MISSEDGOAL, HITOBSTACLE, RESETTING)
    out << 1 << ", " << env.pos_(0) << ", " << env.pos_(1) << ", " << env.obs_(0) << ", " << env.obs_(1) << ", " << env.r_obs_ << ", " << env.goal_(0) << ", " << env.goal_(1) << ", " << RESETTING << "\n";

    // Counter.
    uint c = 0;

    // Average reward.
    bool error = false;

    for (uint i=0;i<n_iter;i++)
    {
        // State of env.
        auto state = env.State().to(device);
        
        // Play.
        double vx_max = 0.5;
        double vy_max = 0.1;
        double wz_max = 0.5;

        auto av = ac->forward(state);

        // Eigen::Vector3d vel(*(actions[c].data<double>()), *(actions[c].data<double>()+1), *(actions[c].data<double>()+2));
        Eigen::Vector3d vel(*(std::get<0>(av).cpu().data<double>()), 0., *(std::get<0>(av).cpu().data<double>()+1));// *(actions[c].cpu().data<double>()+1), *(actions[c].cpu().data<double>()+2));
        // std::cout << vel.transpose() << std::endl;
        auto sd = env.Act(vel);
        env.Reward(std::get<1>(sd));

        // if (i % 300 == 0) {
        //     // Set new goal.
        //     double x_new = double(random_real(gen)); 
        //     double y_new = double(random_real(gen));
        //     env.SetGoal(x_new, y_new);
        // }

        // episode, agent_x, agent_y, goal_x, goal_y, STATUS=(PLAYING, REACHEDGOAL, MISSEDGOAL, HITOBSTACLE, RESETTING)
        out << 1 << ", " << env.pos_(0) << ", " << env.pos_(1) << ", " << env.obs_(0) << ", " << env.obs_(1) << ", " << env.r_obs_ << ", " << env.goal_(0) << ", " << env.goal_(1)  << ", " << std::get<1>(sd) << "\n";
                

        if (std::get<1>(sd) == STATUS::ERROR)
        {
            error = true;

            printf("Quitting episode on unsuccessful return.\n");
            break;
        }
        if (std::get<1>(sd) == STATUS::REACHEDGOAL)
        {
            error = true;

            printf("Quitting episode on victory.\n");
            break;
        }
    }

    out.close();

    return 0;
}

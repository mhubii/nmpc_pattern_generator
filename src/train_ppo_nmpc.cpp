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
    double old_ori_;
    double old_preview_dist_;


    // Constructor.
    NMPCEnvironment(std::string config_file_loc) : nmpc_(config_file_loc), interpol_nmpc_(nmpc_), state_(2)
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
        double x_new = double(random_real(gen)); 
        double y_new = double(random_real(gen));
        SetGoal(x_new, y_new);

        auto theta = nmpc_.Ckq0()(0);
        Eigen::Matrix2d rot;
        rot <<   cos(theta), sin(theta),
                -sin(theta), cos(theta);
        auto goal_trafo = rot*(goal_ - pos_); 

        state_ << goal_trafo;

        old_dist_ = (goal_ -  pos_).norm();
        old_ori_ = goal_trafo.normalized()(0);
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

        double theta = nmpc_.Ckq0()(0);
        Eigen::Matrix2d rot;
        rot <<   cos(theta), sin(theta),
                -sin(theta), cos(theta);
        Eigen::Vector2d goal_trafo;
        goal_trafo = rot*(goal_ - pos_); 

        old_ori_ = goal_trafo.normalized()(0);
        state_ << goal_trafo;

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

        theta = nmpc_.Ckq0()(0);
        rot <<   cos(theta), sin(theta),
                -sin(theta), cos(theta);
        goal_trafo = rot*(goal_ - pos_); 

        state_ << goal_trafo;

        torch::Tensor done = torch::zeros({1, 1}, torch::kF64);
        STATUS status;

        if ((goal_ - pos_).norm() < 4e-1) {
            status = REACHEDGOAL;
            done[0][0] = 1.;
        }
        else if ((goal_ - pos_).norm() > 10.0) {
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

    auto Reward(int status, int iter) -> torch::Tensor
    {
        double goal_factor = 1e1;
        auto dist = (goal_ - pos_).norm();

        auto theta = nmpc_.Ckq0()(0);
        Eigen::Matrix2d rot;
        rot <<   cos(theta), sin(theta),
                -sin(theta), cos(theta);
        auto goal_trafo = rot*(goal_ - pos_); 
        auto ori = goal_trafo.normalized();

        // std::cout << "ori: " << 1e2*goal_factor*(ori(0)-old_ori_) << " dist: " << 5e2*goal_factor*(old_dist_-dist) << " iter: " <<  - 1e-2*iter << std::endl;

        // torch::Tensor reward = torch::full({1, 1}, goal_factor*(old_dist_ - dist), torch::kF64);
        torch::Tensor reward = torch::full({1, 1}, 2e2*goal_factor*(ori(0)-old_ori_)+6e2*goal_factor*(old_dist_-dist) - 1e-2*iter, torch::kF64);

        switch (status)
            {
                case PLAYING:
                    break;
                case REACHEDGOAL:
                    reward[0][0] += 1e2;
                    printf("reached goal, reward: %f\n", *(reward.cpu().data<double>()));
                    break;
                case MISSEDGOAL:
                    reward[0][0] -= 1e3;
                    printf("missed goal, reward: %f\n", *(reward.cpu().data<double>()));
                    break;
                case HITOBSTACLE:
                    reward[0][0] -= 1e3;
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
        double x_new = double(random_real(gen)); 
        double y_new = double(random_real(gen));
        SetGoal(x_new, y_new);
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
        old_ori_ = goal_trafo.normalized()(0);
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
    NMPCEnvironment env(config_file_loc);

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
    double std = 0.1;

    ActorCriticNMPC ac(n_in, n_out, mu_max, std);
    ac->to(torch::kF64);
    // ac->normal(0., 1e-2);
    ac->to(device);
    // torch::optim::Adam opt(ac->parameters(), 0.003);
    torch::optim::Adam opt(ac->parameters(), 0.03);

    // Training loop.
    uint n_iter = 2000;
    uint n_steps = 2000;//20000;
    uint n_agents = uint(n_steps/n_iter);
    uint n_epochs = 50;//500;
    uint mini_batch_size = 200;//;2000;
    uint ppo_epochs = 5;//20;
    double beta = -5e1;

    VT states_pos;
    VT actions;
    VT rewards;
    VT dones;

    VT log_probs;
    VT returns;
    VT values;

    // Output.
    std::ofstream out;
    out.open("../../out/ppo_nmpc/example_ppo.csv");

    // episode, agent_x, agent_y, goal_x, goal_y, STATUS=(PLAYING, REACHEDGOAL, MISSEDGOAL, HITOBSTACLE, RESETTING)
    out << 1 << ", " << env.pos_(0) << ", " << env.pos_(1) << ", " << env.obs_(0) << ", " << env.obs_(1) << ", " << env.r_obs_ << ", " << env.goal_(0) << ", " << env.goal_(1) << ", " << RESETTING << "\n";

    // Counter.
    uint c = 0;

    // Average reward.
    bool error = false;
    double best_avg_rewards = -std::numeric_limits<double>::max();
    double avg_reward = 0.;
    double avg_rewards = 0.;
    uint reward_count = 0;
    double avg_std = 0.;
    double avg_stds = 0.;

    // Save lost history.
    std::ofstream out_loss;
    out_loss.open("../../out/ppo_nmpc/loss_hist.csv");

    for (uint e=0;e<n_epochs;e++)
    {
        printf("epoch %u/%u\n", e+1, n_epochs);

        for (uint i=0;i<n_iter;i++)
        {
            // State of env.
            auto state = env.State();
            states_pos.push_back(state.to(device));
            
            // Play.
            double vx_max = 0.5;
            double vy_max = 0.1;
            double wz_max = 0.5;

            auto av = ac->forward(states_pos[c]);
            actions.push_back(std::get<0>(av));
            values.push_back(std::get<1>(av));
            log_probs.push_back(ac->log_prob(actions[c]));

            // Eigen::Vector3d vel(*(actions[c].data<double>()), *(actions[c].data<double>()+1), *(actions[c].data<double>()+2));
            Eigen::Vector3d vel(*(actions[c].cpu().data<double>()), 0., *(actions[c].cpu().data<double>()+1));// *(actions[c].cpu().data<double>()+1), *(actions[c].cpu().data<double>()+2));
            // std::cout << vel.transpose() << std::endl;
            auto sd = env.Act(vel);

            if (std::get<1>(sd) == STATUS::ERROR)
            {
                error = true;

                printf("Quitting episode on unsuccessful return.\n");
                // c = 0;

                // states_pos.clear();
                // actions.clear();
                // rewards.clear();
                // dones.clear();

                // log_probs.clear();
                // returns.clear();
                // values.clear();

                break;
            }
            else
            {
                // New state.
                rewards.push_back(env.Reward(std::get<1>(sd), i).to(device));
                dones.push_back(std::get<2>(sd).to(device));

                avg_reward += *(rewards[c].cpu().data<double>())/n_iter;
                avg_std += *(ac->log_std_.exp().cpu().data<double>())/n_iter;

                // episode, agent_x, agent_y, goal_x, goal_y, STATUS=(PLAYING, REACHEDGOAL, MISSEDGOAL, HITOBSTACLE, RESETTING)
                out << e+1 << ", " << env.pos_(0) << ", " << env.pos_(1) << ", " << env.obs_(0) << ", " << env.obs_(1) << ", " << env.r_obs_ << ", " << env.goal_(0) << ", " << env.goal_(1)  << ", " << std::get<1>(sd) << "\n";
                
                if (*(dones[c].cpu().data<double>()) == 1.) 
                {
                    // Reset the environment.
                    env.Reset();

                    // episode, agent_x, agent_y, goal_x, goal_y, STATUS=(PLAYING, REACHEDGOAL, MISSEDGOAL, HITOBSTACLE, RESETTING)
                    out << e+1 << ", " << env.pos_(0) << ", " << env.pos_(1) << ", " << env.obs_(0) << ", " << env.obs_(1) << ", " << env.r_obs_ << ", " << env.goal_(0) << ", " << env.goal_(1)  << ", " << RESETTING << "\n";
                }

                c++;

                // Update.
                if (c%n_steps == 0)
                {
                    printf("Updating network.\n");
                    values.push_back(std::get<1>(ac->forward(states_pos[c-1])));

                    returns = PPO::returns(rewards, dones, values, .99, .95);

                    torch::Tensor t_log_probs = torch::cat(log_probs).detach();
                    torch::Tensor t_returns = torch::cat(returns).detach();
                    torch::Tensor t_values = torch::cat(values).detach();
                    torch::Tensor t_states_pos = torch::cat(states_pos);
                    torch::Tensor t_actions = torch::cat(actions);
                    torch::Tensor t_advantages = t_returns - t_values.slice(0, 0, n_steps);

                    // beta = -1e2/std::abs(avg_reward);
                    PPO::update(ac, t_states_pos, t_actions, t_log_probs, t_returns, t_advantages, opt, n_steps, ppo_epochs, mini_batch_size, beta);
                
                    c = 0;

                    states_pos.clear();
                    actions.clear();
                    rewards.clear();
                    dones.clear();

                    log_probs.clear();
                    returns.clear();
                    values.clear();
                }
            }
        }

        if (!error) {

            avg_rewards += avg_reward;
            avg_stds += avg_std;
            reward_count++;
            if (reward_count % n_agents == 0)
            {
                avg_rewards /= double(reward_count);
                avg_stds /= double(reward_count);
                reward_count = 0;
                out_loss << e << ", " << avg_rewards << ", " << avg_stds << "\n";

                // Save the best net.
                if (avg_rewards > best_avg_rewards) {

                    best_avg_rewards = avg_rewards;
                    printf("Best average reward: %f\n", best_avg_rewards);
                    torch::save(ac, "../../out/ppo_nmpc/ppo_nmpc_best_model.pt");
                }
                printf("Average reward: %f at entropy %f\n", avg_rewards, avg_stds);
                avg_rewards = 0.;
                avg_stds = 0.;
            }
        }
        error = false;
        avg_reward = 0.;
        avg_std = 0.;

        // Reset the environment.
        env.Reset();

        // episode, agent_x, agent_y, goal_x, goal_y, STATUS=(PLAYING, REACHEDGOAL, MISSEDGOAL, HITOBSTACLE, RESETTING)
        out << e+1 << ", " << env.pos_(0) << ", " << env.pos_(1) << ", " << env.obs_(0) << ", " << env.obs_(1) << ", " << env.r_obs_ << ", " << env.goal_(0) << ", " << env.goal_(1)  << ", " << RESETTING << "\n";
    }

    out_loss.close();
    out.close();

    return 0;
}

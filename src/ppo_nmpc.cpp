#include <Eigen/Core>
#include <qpOASES.hpp>

#include "nmpc_generator.h"
#include "interpolation.h"
#include "utils.h"
#include "proximal_policy_optimization.h"
#include "timer.h"

enum STATUS {
    PLAYING,
    REACHEDGOAL,
    MISSEDGOAL,
    HITOBSTACLE,
    RESETTING,
    ERROR
};

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
    NMPCEnvironment(std::string config_file_loc) : nmpc_(config_file_loc), interpol_nmpc_(nmpc_), state_(4)
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
        goal_.setZero();
        state_ << pos_, goal_;

        old_dist_ = (goal_ - pos_).norm();
        old_preview_dist_ = PreviewDist();
    };

    auto State() -> torch::Tensor
    {
        torch::Tensor state = torch::zeros({1, state_.size()}, torch::kF64);
        std::memcpy(state.data_ptr(), state_.data(), state_.size()*sizeof(double));
        return state;
    }

    // Functions to interact with the environment for reinforcement learning.
    auto Act(Eigen::Vector3d vel) -> std::tuple<torch::Tensor /*state*/, int, torch::Tensor>
    {
        old_dist_ = (goal_ - pos_).norm();
        old_preview_dist_ = PreviewDist();

        // Run nmpc for one episode.
        vel_ += vel;

        nmpc_.SetVelocityReference(vel_);
        nmpc_.Solve();
        nmpc_.Simulate();
        interpol_nmpc_.InterpolateStep();

        pg_state_ = nmpc_.Update();
        nmpc_.SetInitialValues(pg_state_);

        // Update state.
        pos_ << nmpc_.Ckx0()[0], nmpc_.Cky0()[0];
        vel_ << nmpc_.LocalVelRef();
        state_ << pos_, goal_;

        // Check game targets.
        torch::Tensor state = State();
        torch::Tensor done = torch::zeros({1, 1}, torch::kF64);
        STATUS status;

        if ((goal_ - pos_).norm() < 3e-1) {
            status = REACHEDGOAL;
            done[0][0] = 1.;
        }
        else if ((goal_ - pos_).norm() > 4.0) {
            status = MISSEDGOAL;
            done[0][0] = 1.;
        }
        else if ((obs_ - pos_).norm() < nmpc_.RObs()) {
            status = HITOBSTACLE;
            done[0][0] = 1.;
        }
        else {
            status = PLAYING;
            done[0][0] = 0.;
        }
        if (nmpc_.GetStatus() != qpOASES::SUCCESSFUL_RETURN) {
            status = ERROR;
            done[0][0] = 0.;
        }

        return std::make_tuple(state, status, done);
    };

    auto Reward(int status) -> torch::Tensor
    {
        double goal_factor = 1e2;
        torch::Tensor reward = torch::full({1, 1}, goal_factor*(old_preview_dist_ - PreviewDist()), torch::kF64);

        switch (status)
            {
                case PLAYING:
                    break;
                case REACHEDGOAL:
                    reward[0][0] += 1e2;
                    printf("reached goal, reward: %f\n", *(reward.cpu().data<double>()));
                    break;
                case MISSEDGOAL:
                    reward[0][0] -= 1e1;
                    printf("missed goal, reward: %f\n", *(reward.cpu().data<double>()));
                    break;
                case HITOBSTACLE:
                    reward[0][0] -= 1e1;
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
        state_ << pos_, goal_;
    };

    auto SetGoal(Eigen::Vector2d& goal) -> void
    {
        goal_(0) = goal(0);
        goal_(1) = goal(1);

        old_dist_ = (goal_ - pos_).norm();
        old_preview_dist_ = PreviewDist();
        state_ << pos_, goal_;
    };

    auto SetObstacle(Eigen::Vector2d obs, double r) -> void
    {
        // obs_(0) = obs(0);
        // obs_(1) = obs(1);
        obs_(0) = 10.;
        obs_(1) = 0.;

        r_obs_ = r;
        double r_margin = nmpc_.RMargin();

        Circle c{obs(0), obs(1), r, r_margin};

        nmpc_.SetObstacle(c);

        // Update state.
        state_ << pos_, goal_;
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

    // Random engine for spawning the goal and the obstacle.
    std::random_device rd;
    std::mt19937 re(rd());
    std::uniform_real_distribution<> dist(-M_PI/8., M_PI/8.);

    double angle = dist(re);
    Eigen::Vector2d goal(2*cos(angle), 2*sin(angle)); // spawn goal on a circle
    angle = dist(re);
    Eigen::Vector2d obs(cos(angle), sin(angle));
    double r_obs = 0.1;

    env.SetGoal(goal);
    env.SetObstacle(obs, r_obs);

    // Proximal policy optimization. We use informations of the states system as well as its environment for n_in.
    torch::DeviceType device_type;
    if (torch::cuda::is_available()) {
        printf("CUDA available! Training on GPU.\n");
        device_type = torch::kCUDA;
    } 
    else {
        printf("Training on CPU.\n");
        device_type = torch::kCPU;
    }
    torch::Device device(device_type);

    uint n_in = env.state_.size();
    uint n_out = 2;//env.vel_.size();
    double mu_max = 1e-1;
    double std = 1e-2;

    ActorCritic ac(n_in, n_out, mu_max, std);
    ac->to(torch::kF64);
    ac->normal(0., 1e-2);
    ac->to(device);
    torch::optim::Adam opt(ac->parameters(), 1e-3);

    // Training loop.
    uint n_iter = 5000;
    uint n_steps = 1250;
    uint n_epochs = 10;
    uint mini_batch_size = 4;
    uint ppo_epochs = 4;
    double beta = 1e-3;

    VT states;
    VT actions;
    VT rewards;
    VT dones;

    VT log_probs;
    VT returns;
    VT values;

    // Output.
    std::ofstream out;
    out.open("example_ppo.csv");

    // episode, agent_x, agent_y, goal_x, goal_y, STATUS=(PLAYING, REACHEDGOAL, MISSEDGOAL, HITOBSTACLE, RESETTING)
    out << 1 << ", " << env.pos_(0) << ", " << env.pos_(1) << ", " << env.obs_(0) << ", " << env.obs_(1) << ", " << env.r_obs_ << ", " << env.goal_(0) << ", " << env.goal_(1) << ", " << RESETTING << "\n";

    // Counter.
    uint c = 0;

    // Average reward.
    bool error = false;
    double best_avg_reward = -std::numeric_limits<double>::max();
    double avg_reward = 0.;
    double avg_entropy = 0.;

    // Save lost history.
    std::ofstream out_loss;
    out_loss.open("ppo_nmpc_loss_hist.csv");

    for (uint e=0;e<n_epochs;e++)
    {
        printf("epoch %u/%u\n", e+1, n_epochs);

        for (uint i=0;i<n_iter;i++)
        {
            // State of env.
            states.push_back(env.State().to(device));
            
            // Play.
            double vx_max = 0.02;
            double vy_max = 0.002;
            double wz_max = 0.01;

            auto av = ac->forward(states[c]);
            actions.push_back(std::get<0>(av));
            values.push_back(std::get<1>(av));
            log_probs.push_back(ac->log_prob(actions[c]));

            // Eigen::Vector3d vel(*(actions[c].data<double>()), *(actions[c].data<double>()+1), *(actions[c].data<double>()+2));
            Eigen::Vector3d vel(*(actions[c].cpu().data<double>())*vx_max, *(actions[c].cpu().data<double>()+1)*vy_max, 0.);// *(actions[c].cpu().data<double>()+1), *(actions[c].cpu().data<double>()+2));
            auto sd = env.Act(vel);

            if (std::get<1>(sd) == ERROR)
            {
                error = true;

                printf("Quitting episode on unsuccessful return.\n");
                c = 0;

                states.clear();
                actions.clear();
                rewards.clear();
                dones.clear();

                log_probs.clear();
                returns.clear();
                values.clear();

                break;
            }
            else
            {
                // New state.
                rewards.push_back(env.Reward(std::get<1>(sd)).to(device));
                dones.push_back(std::get<2>(sd).to(device));

                avg_reward += *(rewards[c].cpu().data<double>())/n_iter;
                avg_entropy += *(ac->entropy().cpu().data<double>())/n_iter;

                // episode, agent_x, agent_y, goal_x, goal_y, STATUS=(PLAYING, REACHEDGOAL, MISSEDGOAL, HITOBSTACLE, RESETTING)
                out << e+1 << ", " << env.pos_(0) << ", " << env.pos_(1) << ", " << env.obs_(0) << ", " << env.obs_(1) << ", " << env.r_obs_ << ", " << env.goal_(0) << ", " << env.goal_(1)  << ", " << std::get<1>(sd) << "\n";
                
                if (*(dones[c].cpu().data<double>()) == 1.) 
                {
                    // Reset the environment.
                    env.Reset();

                    // Set a new goal and a new obstacle.
                    angle = dist(re);
                    goal << 2*cos(angle), 2*sin(angle);
                    angle = dist(re);
                    obs << cos(angle), sin(angle);

                    env.SetGoal(goal);
                    env.SetObstacle(obs, r_obs);

                    // episode, agent_x, agent_y, goal_x, goal_y, STATUS=(PLAYING, REACHEDGOAL, MISSEDGOAL, HITOBSTACLE, RESETTING)
                    out << e+1 << ", " << env.pos_(0) << ", " << env.pos_(1) << ", " << env.obs_(0) << ", " << env.obs_(1) << ", " << env.r_obs_ << ", " << env.goal_(0) << ", " << env.goal_(1)  << ", " << RESETTING << "\n";
                }

                c++;

                // Update.
                if (c%n_steps == 0)
                {
                    values.push_back(std::get<1>(ac->forward(states[c-1])));

                    returns = PPO::returns(rewards, dones, values, .99, .95);

                    torch::Tensor t_log_probs = torch::cat(log_probs).detach();
                    torch::Tensor t_returns = torch::cat(returns).detach();
                    torch::Tensor t_values = torch::cat(values).detach();
                    torch::Tensor t_states = torch::cat(states);
                    torch::Tensor t_actions = torch::cat(actions);
                    torch::Tensor t_advantages = t_returns - t_values.slice(0, 0, n_steps);

                    printf("Updating network.\n");
                    PPO::update(ac, t_states, t_actions, t_log_probs, t_returns, t_advantages, opt, n_steps, ppo_epochs, mini_batch_size, beta);
                
                    c = 0;

                    states.clear();
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

            out_loss << e << ", " << avg_reward << ", " << avg_entropy << "\n";

            // Save the best net.
            if (avg_reward > best_avg_reward) {

                best_avg_reward = avg_reward;
                printf("Best average reward: %f\n", best_avg_reward);
                torch::save(ac, "ppo_nmpc_best_model.pt");
            }

            printf("Average reward: %f at entropy %f\n", avg_reward, avg_entropy);
        }
        error = false;
        avg_reward = 0.;
        avg_entropy = 0.;

        // Reset the environment.
        env.Reset();

        // Set a new goal and a new obstacle.
        angle = dist(re);
        goal << 2*cos(angle), 2*sin(angle);
        angle = dist(re);
        obs << cos(angle), sin(angle);

        env.SetGoal(goal);
        env.SetObstacle(obs, r_obs);

        // episode, agent_x, agent_y, goal_x, goal_y, STATUS=(PLAYING, REACHEDGOAL, MISSEDGOAL, HITOBSTACLE, RESETTING)
        out << e+1 << ", " << env.pos_(0) << ", " << env.pos_(1) << ", " << env.obs_(0) << ", " << env.obs_(1) << ", " << env.r_obs_ << ", " << env.goal_(0) << ", " << env.goal_(1)  << ", " << RESETTING << "\n";
    }

    out_loss.close();
    out.close();

    return 0;
}

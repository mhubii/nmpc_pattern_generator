#include <Eigen/Core>

#include "nmpc_generator.h"
#include "interpolation.h"
#include "utils.h"
#include "proximal_policy_optimization.h"
#include "timer.h"

enum AGENT {
    PLAYING,
    REACHEDGOAL,
    MISSEDGOAL,
    HITOBSTACLE,
    RESETTING
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

    // Constructor.
    NMPCEnvironment(std::string config_file_loc) : nmpc_(config_file_loc), interpol_nmpc_(nmpc_), state_(10)
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
        vel_ << nmpc_.Ckx0()[1], nmpc_.Cky0()[1], nmpc_.Ckq0()[1];
        obs_ << nmpc_.XObs(), nmpc_.YObs();
        r_obs_ = nmpc_.RObs();
        goal_.setZero();
        state_ << pos_, vel_, obs_, r_obs_, goal_;

        old_dist_ = (goal_ - pos_).norm();
    };

    // Functions to interact with the environment for reinforcement learning.
    auto Act(Eigen::Vector3d vel) -> std::tuple<Eigen::VectorXd /*state*/, AGENT>
    {
        old_dist_ = (goal_ - pos_).norm();

        // Run nmpc for one episode.
        nmpc_.SetVelocityReference(vel);
        nmpc_.Solve();
        nmpc_.Simulate();
        interpol_nmpc_.InterpolateStep();

        pg_state_ = nmpc_.Update();
        nmpc_.SetInitialValues(pg_state_);

        // Update state.
        pos_ << nmpc_.Ckx0()[0], nmpc_.Cky0()[0];
        vel_ << nmpc_.Ckx0()[1], nmpc_.Cky0()[1], nmpc_.Ckq0()[1];
        state_ << pos_, vel_, obs_, r_obs_, goal_;

        // Check game targets.
        AGENT agent;

        if ((goal_ - pos_).norm() < 4e-1) {
            agent = REACHEDGOAL;
        }
        else if ((goal_ - pos_).norm() > 1e1) {
            agent = MISSEDGOAL;
        }
        else if ((obs_ - pos_).norm() < nmpc_.RObs()) {
            agent = HITOBSTACLE;
        }
        else {
            agent = PLAYING;
        }

        return std::make_tuple(state_, agent);
    };

    auto Reward() -> double
    {
        return old_dist_ - (goal_ - pos_).norm();
    };

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
        vel_ << nmpc_.Ckx0()[1], nmpc_.Cky0()[1], nmpc_.Ckq0()[1];
        state_ << pos_, vel_, obs_, r_obs_, goal_;
    };

    auto SetGoal(Eigen::Vector2d& goal) -> void
    {
        goal_(0) = goal(0);
        goal_(1) = goal(1);

        old_dist_ = (goal_ - pos_).norm();
        state_ << pos_, vel_, obs_, r_obs_, goal_;
    };

    auto SetObstacle(Eigen::Vector2d obs, double r) -> void
    {
        obs_(0) = obs(0);
        obs_(1) = obs(1);
        r_obs_ = r;
        double r_margin = nmpc_.RMargin();

        Circle c{obs(0), obs(1), r, r_margin};

        nmpc_.SetObstacle(c);

        // Update state.
        state_ << pos_, vel_, obs_, r_obs_, goal_;
    };
};


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
    // Setup the nonlinear model predictive control.
    std::string config_file_loc = argv[1];
    NMPCEnvironment env(config_file_loc);

    // Random engine for spawning the goal and the obstacle.
    std::random_device rd;
    std::mt19937 re(rd());
    std::uniform_int_distribution<> dist_goal(-5, 5);
    std::uniform_real_distribution<> dist_obs(-1, 1); // spawn obstacle in the center of a line between spawn and goal

    Eigen::Vector2d goal(double(dist_goal(re)), double(dist_goal(re)));
    Eigen::Vector2d line = goal - env.pos_;
    Eigen::Vector2d obs = line - 0.5*line + 0.5*line.norm()*dist_obs(re)*line.normalized();
    double r_obs = 0.1;

    env.SetGoal(goal);
    env.SetObstacle(obs, r_obs);

    // Proximal policy optimization. We use informations of the states system as well as its environment for n_in.
    uint n_in = env.state_.size();
    uint n_out = env.vel_.size();
    double std = 1e-2;

    ActorCritic ac(n_in, n_out, std);
    ac.to(torch::kF64);
    ac.normal(0., 1e-2);
    torch::optim::Adam opt(ac.parameters(), 1e-3);

    // Training loop.
    uint n_iter = 1000;
    uint n_steps = 64;
    uint n_epochs = 20;
    uint mini_batch_size = 16;
    uint ppo_epochs = uint(n_steps/mini_batch_size);

    torch::Tensor state = torch::zeros({1, n_in}, torch::kF64);
    torch::Tensor action = torch::zeros({1, n_out}, torch::kF64);
    torch::Tensor reward = torch::zeros({1, 1}, torch::kF64);
    torch::Tensor next_state = torch::zeros({1, n_in}, torch::kF64);
    torch::Tensor done = torch::zeros({1, 1}, torch::kF64);

    torch::Tensor log_prob = torch::zeros({1, 1}, torch::kF64);
    torch::Tensor value = torch::zeros({1, 1}, torch::kF64);

    VT states(n_steps, torch::zeros({1, n_in}, torch::kF64));
    VT actions(n_steps, torch::zeros({1, n_out}, torch::kF64));
    VT rewards(n_steps, torch::zeros({1, 1}, torch::kF64));
    VT next_states(n_steps, torch::zeros({1, n_in}, torch::kF64));
    VT dones(n_steps, torch::zeros({1, 1}, torch::kF64));

    VT log_probs(n_steps, torch::zeros({1, n_out}, torch::kF64));
    VT returns(n_steps, torch::zeros({1, 1}, torch::kF64));
    VT values(n_steps+1, torch::zeros({1, 1}, torch::kF64));

    // Output.
    std::ofstream out;
    out.open("example_ppo.csv");

    // Initial system's state.
    for (uint i=0;i<n_in;i++)
    {
        state[0][i] = env.state_(i);
    }

    // episode, agent_x, agent_y, goal_x, goal_y, AGENT=(PLAYING, WON, LOST)
    out << 1 << ", " << env.pos_(0) << ", " << env.pos_(1) << ", " << env.goal_(0) << ", " << env.goal_(1) << ", " << RESETTING << "\n";

    // Counter.
    uint c = 0;

    for (uint e=0;e<n_epochs;e++)
    {
        printf("epoch %u/%u\n", e+1, n_epochs);

        for (uint i=0;i<n_iter;i++)
        {
            // Play.
            auto av = ac.forward(state);
            action = std::get<0>(av);
            value = std::get<1>(av);
            log_prob = ac.log_prob(action);

            Eigen::Vector3d vel(*(action.data<double>()), *(action.data<double>()+1), *(action.data<double>()+2));
            auto sd = env.Act(vel);

            // New state.
            reward[0][0] = env.Reward();
            for (uint i=0;i<n_in;i++)
            {
                next_state[0][i] = std::get<0>(sd)(i);
            }
            switch (std::get<1>(sd))
            {
                case PLAYING:
                    done[0][0] = 0.;
                    break;
                case REACHEDGOAL:
                    reward[0][0] += 10.;
                    done[0][0] = 1.;
                    printf("reached goal, reward: %f\n", *(reward.data<double>()));
                    break;
                case MISSEDGOAL:
                    reward[0][0] -= 10.;
                    done[0][0] = 1.;
                    printf("missed goal, reward: %f\n", *(reward.data<double>()));
                    break;
                case HITOBSTACLE:
                    reward[0][0] -= 10.;
                    done[0][0] = 1.;
                    printf("hit obstacle, reward: %f\n", *(reward.data<double>()));
                    break;
            }

            // episode, agent_x, agent_y, goal_x, goal_y, AGENT=(PLAYING, WON, LOST)
            out << e+1 << ", " << env.pos_(0) << ", " << env.pos_(1) << ", " << env.goal_(0) << ", " << env.goal_(1) << ", " << std::get<1>(sd) << "\n";


            // Store everything.
            states[c].copy_(state);
            rewards[c].copy_(reward);
            actions[c].copy_(action);
            next_states[c].copy_(next_state);
            dones[c].copy_(done);

            log_probs[c].copy_(log_prob);
            values[c].copy_(value);
            
            c++;

            // Update.
            if (c%n_steps == 0)
            {
                values[c] = std::get<1>(ac.forward(next_state));

                returns = PPO::returns(rewards, dones, values, .99, .95);

                torch::Tensor t_log_probs = torch::cat(log_probs).detach();
                torch::Tensor t_returns = torch::cat(returns).detach();
                torch::Tensor t_values = torch::cat(values).detach();
                torch::Tensor t_states = torch::cat(states);
                torch::Tensor t_actions = torch::cat(actions);
                torch::Tensor t_advantages = t_returns - t_values.slice(0, 0, n_steps);

                PPO::update(ac, t_states, t_actions, t_log_probs, t_returns, t_advantages, opt, n_steps, ppo_epochs, mini_batch_size);
            
                c = 0;
            }

            state.copy_(next_state);

            if (*(done.data<double>()) == 1.) 
            {
                // Reset the environment.
                env.Reset();

                // Set a new goal and a new obstacle.
                goal << double(dist_goal(re)), double(dist_goal(re));
                line = goal - env.pos_;
                obs = line - 0.5*line + 0.5*line.norm()*dist_obs(re)*line.normalized();

                env.SetGoal(goal);
                env.SetObstacle(obs, r_obs);

                // Initial state of env.
                for (uint i=0;i<n_in;i++)
                {
                    state[0][i] = env.state_(i);
                }
            }
        }

        // Reset the environment.
        env.Reset();

        // Set a new goal and a new obstacle.
        goal << double(dist_goal(re)), double(dist_goal(re));
        line = goal - env.pos_;
        obs = line - 0.5*line + 0.5*line.norm()*dist_obs(re)*line.normalized();

        env.SetGoal(goal);
        env.SetObstacle(obs, r_obs);

        // Initial state of env.
        for (uint i=0;i<n_in;i++)
        {
            state[0][i] = env.state_(i);
        }

        // episode, agent_x, agent_y, goal_x, goal_y, AGENT=(PLAYING, WON, LOST)
        out << e+1 << ", " << env.pos_(0) << ", " << env.pos_(1) << ", " << env.goal_(0) << ", " << env.goal_(1) << ", " << RESETTING << "\n";
    }

    return 0;
}
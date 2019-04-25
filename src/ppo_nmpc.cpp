#include <Eigen/Core>
#include <qpOASES.hpp>
#include <random>
#include <cmath>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <torch/torch.h>

#include "nmpc_generator.h"
#include "interpolation.h"
#include "utils.h"
#include "proximal_policy_optimization.h"
#include "timer.h"

std::random_device rdev;  //Will be used to obtain a seed for the random number engine
std::mt19937 gen(rdev()); //Standard mersenne_twister_engine seeded with rd()
std::uniform_real_distribution<> random_real(0., 1.); // generate a random boolean

enum STATUS {
    PLAYING,
    REACHEDGOAL,
    MISSEDGOAL,
    HITOBSTACLE,
    HITWALL,
    RESETTING,
    ERROR
};

// Create a gaussian kernel for an obstacle in the map.
cv::Mat getGaussianKernel(int rows, int cols, double sigmax, double sigmay)
{
    const auto y_mid = (rows-1) / 2.0;
    const auto x_mid = (cols-1) / 2.0;

    const auto x_spread = 1. / (sigmax*sigmax*2);
    const auto y_spread = 1. / (sigmay*sigmay*2);

    const auto denominator = 1./255.;

    std::vector<double> gauss_x, gauss_y;

    gauss_x.reserve(cols);
    for (auto i = 0;  i < cols;  ++i) {
        auto x = i - x_mid;
        gauss_x.push_back(std::exp(-x*x * x_spread));
    }

    gauss_y.reserve(rows);
    for (auto i = 0;  i < rows;  ++i) {
        auto y = i - y_mid;
        gauss_y.push_back(std::exp(-y*y * y_spread));
    }

    cv::Mat kernel = cv::Mat::zeros(rows, cols, CV_8U);
    for (auto j = 0;  j < rows;  ++j)
        for (auto i = 0;  i < cols;  ++i) {
            kernel.at<uchar>(j,i) = gauss_x[i] * gauss_y[j] / denominator;
        }

    return kernel;
}

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

    // Environment.
    double x_size_; // x and y size of map in m
    double y_size_;
    double dx_;     // map resolution
    double dy_;
    int nx_;        // number of pixels
    int ny_;
    int x_off_; // offset of the com in the image
    int y_off_;
    cv::Mat map_; // map of the environment


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

        x_size_ = 6;
        y_size_ = 6;
        dx_ = 1/100.; // cm resolution
        dy_ = dx_;
	    nx_ = int(x_size_/dx_);
	    ny_ = int(y_size_/dy_);
        x_off_ = int(nx_/4.);
        y_off_ = int(ny_/2.);

	    map_ = cv::Mat(cv::Size(nx_, ny_), CV_8U, cv::Scalar(255));

        // Initialize the map.
        InitializeMap(); // with obstacle initially being at the left
    };

    auto State() -> std::tuple<torch::Tensor, torch::Tensor>
    {
        // Return position and goal.
        torch::Tensor state = torch::zeros({1, state_.size()}, torch::kF64);
        std::memcpy(state.data_ptr(), state_.data(), state_.size()*sizeof(double));

        // Return the surrounding parts of the map, 64x64 centered at the current position.
        int x = x_off_ + pos_[0]/dx_; // offset (arbitrarily chosen) + current position
        int y = y_off_ + pos_[1]/dy_;
        cv::Mat crop = cv::Mat(map_, cv::Rect(x - int(64/2), y - int(64/2), 64, 64)).clone(); 
        torch::Tensor map = torch::from_blob(crop.data, {1, crop.rows, crop.cols, 1}, torch::kByte);

        map = map.permute({0, 3, 1, 2}); // hxwxc -> cxhxw
        map = map.to(torch::kF64);

        return std::make_tuple(state, map);
    }

    // Functions to interact with the environment for reinforcement learning.
    auto Act(Eigen::Vector3d vel) -> std::tuple<torch::Tensor /*position*/, torch::Tensor /*map*/, int, torch::Tensor>
    {
        old_dist_ = (goal_ - pos_).norm();
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
        state_ << pos_, goal_;

        // Check game targets.
        auto state = State();
        torch::Tensor pos = std::get<0>(state);
        torch::Tensor map = std::get<1>(state);

        torch::Tensor done = torch::zeros({1, 1}, torch::kF64);
        STATUS status;

        // If hit wall -> end

        if ((goal_ - pos_).norm() < 4e-1) {
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
        else if ((40 < pos_[0]/dx_ || pos_[0]/dx_ < nx_-40) || (40 < pos_[1]/dy_ || pos_[1]/dy_ < ny_-40)) { 
            status = HITWALL;
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

        return std::make_tuple(pos, map, status, done);
    };

    auto Reward(int status) -> torch::Tensor
    {
        double goal_factor = 1e2;
        torch::Tensor reward = torch::full({1, 1}, goal_factor*(old_preview_dist_ - PreviewDist()), torch::kF64);

        // Negative reward for touching obstacles and walls.
        reward[0][0] -= double(map_.at<uchar>(y_off_ + pos_[1]/dy_, x_off_ + pos_[0]/dx_));

        switch (status)
            {
                case PLAYING:
                    break;
                case REACHEDGOAL:
                    reward[0][0] += 1e3;
                    printf("reached goal, reward: %f\n", *(reward.cpu().data<double>()));
                    break;
                case MISSEDGOAL:
                    reward[0][0] -= 1e1;
                    printf("missed goal, reward: %f\n", *(reward.cpu().data<double>()));
                    break;
                case HITOBSTACLE:
                    reward[0][0] -= 1e2;
                    printf("hit obstacle, reward: %f\n", *(reward.cpu().data<double>()));
                    break;
                // case HITWALL:
                //     reward[0][0] -= 1e2;
                //     printf("hit wall, reward: %f\n", *(reward.cpu().data<double>()));
                //     break;
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

        SetGoal();       // Set the goal at the end of the roi, in the nmpc frame
        SetObstacle();   // Set obstacle for the nmpc
        InitializeMap(); // Initialize the map
    };

    auto SetGoal() -> void
    {
        int roi_x_size = int(nx_*2./3.);
        goal_(0) = 3.*double(roi_x_size - 2*x_off_)*dx_;
        goal_(1) = 0.;

        old_dist_ = (goal_ - pos_).norm();
        old_preview_dist_ = PreviewDist();
        state_ << pos_, goal_;
    };

    auto SetObstacle() -> void
    {
	    // Create central path.
        int roi_x_size = int(nx_*2./3.);
	    int roi_y_size = int(ny_/2.);
        int roi_x_pos = (nx_ - roi_x_size)/2.;
        int roi_y_pos = (ny_ - roi_y_size)/2.;

        roi_x_pos += int(roi_x_size/2.); // compute the center of the gaussian distribution in image coordinates
        roi_y_pos += int(roi_y_size/2.);

        obs_(0) = (roi_x_pos - x_off_)*dx_;
        obs_(1) = (roi_y_pos - y_off_)*dy_;

        r_obs_ = roi_y_size/8.*dx_; // same size as sigmax (maybe a little smaller)  //double(roi_y_size)/4.*dx_; // roughly size of the roi/4
        double r_margin = nmpc_.RMargin();

        Circle c{obs_(0), obs_(1), r_obs_, r_margin};

        nmpc_.SetObstacle(c);

        // Update state.
        state_ << pos_, goal_;
    };

    auto InitializeMap() -> void
    {
	    // Create central path.
        int roi_x = int(nx_*2./3.);
	    int roi_y = int(ny_/2.);

        cv::Mat roi = map_(cv::Rect((nx_ - roi_x)/2., (ny_ - roi_y)/2., roi_x, roi_y));
        roi.setTo(0);

        // Set gaussian at obstacle.
        cv::Mat roi_g = roi(cv::Rect(roi.cols/2. - roi.rows/3., roi.rows/2. - roi.rows/3., roi.rows*2./3., roi.rows*2./3.));
        cv::Mat gaussian = getGaussianKernel(roi.rows*2./3., roi.rows*2./3., roi.rows/8., roi.rows/8.);
        gaussian.copyTo(roi_g);
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
    uint height = 64;
    uint width = 64;
    uint n_out = 2;//env.vel_.size();
    double mu_max = 1e-1;
    double std = 1e-2;

    ActorCriticNMPC ac(n_in, height, width, n_out, mu_max, std);
    ac->to(torch::kF64);
    ac->normal(0., 1e-2);
    ac->to(device);
    torch::optim::Adam opt(ac->parameters(), 1e-4);

    // Training loop.
    uint n_iter = 4000;
    uint n_steps = 600;
    uint n_epochs = 100;
    uint mini_batch_size = 100;
    uint ppo_epochs = 6;
    double beta = 1e-4;

    VT states_pos;
    VT states_map;
    VT actions;
    VT rewards;
    VT dones;

    VT log_probs;
    VT returns;
    VT values;

    // Output.
    std::ofstream out;
    out.open("../../out/ppo_nmpc/example_ppo.csv");

    // episode, agent_x, agent_y, goal_x, goal_y, STATUS=(PLAYING, REACHEDGOAL, MISSEDGOAL, HITOBSTACLE, HITWALL, RESETTING)
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
            auto state = env.State();
            states_pos.push_back(std::get<0>(state).to(device));
            states_map.push_back(std::get<1>(state).to(device));
            
            // Play.
            double vx_max = 1.0;
            double vy_max = 0.1;
            double wz_max = 0.01;

            auto av = ac->forward(states_pos[c], states_map[c]);
            actions.push_back(std::get<0>(av));
            values.push_back(std::get<1>(av));
            log_probs.push_back(ac->log_prob(actions[c]));

            // Eigen::Vector3d vel(*(actions[c].data<double>()), *(actions[c].data<double>()+1), *(actions[c].data<double>()+2));
            Eigen::Vector3d vel(*(actions[c].cpu().data<double>())*vx_max, *(actions[c].cpu().data<double>()+1)*vy_max, 0.);// *(actions[c].cpu().data<double>()+1), *(actions[c].cpu().data<double>()+2));
            auto sd = env.Act(vel);

            if (std::get<2>(sd) == STATUS::ERROR)
            {
                error = true;

                printf("Quitting episode on unsuccessful return.\n");
                c = 0;

                states_pos.clear();
                states_map.clear();
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
                rewards.push_back(env.Reward(std::get<2>(sd)).to(device));
                dones.push_back(std::get<3>(sd).to(device));

                avg_reward += *(rewards[c].cpu().data<double>())/n_iter;
                avg_entropy += *(ac->entropy().cpu().data<double>())/n_iter;

                // episode, agent_x, agent_y, goal_x, goal_y, STATUS=(PLAYING, REACHEDGOAL, MISSEDGOAL, HITOBSTACLE, HITWALL, RESETTING)
                out << e+1 << ", " << env.pos_(0) << ", " << env.pos_(1) << ", " << env.obs_(0) << ", " << env.obs_(1) << ", " << env.r_obs_ << ", " << env.goal_(0) << ", " << env.goal_(1)  << ", " << std::get<2>(sd) << "\n";
                
                if (*(dones[c].cpu().data<double>()) == 1.) 
                {
                    // Reset the environment.
                    env.Reset();

                    // episode, agent_x, agent_y, goal_x, goal_y, STATUS=(PLAYING, REACHEDGOAL, MISSEDGOAL, HITOBSTACLE, HITWALL, RESETTING)
                    out << e+1 << ", " << env.pos_(0) << ", " << env.pos_(1) << ", " << env.obs_(0) << ", " << env.obs_(1) << ", " << env.r_obs_ << ", " << env.goal_(0) << ", " << env.goal_(1)  << ", " << RESETTING << "\n";
                }

                c++;

                // Update.
                if (c%n_steps == 0)
                {
                    // printf("Updating network.\n");
                    values.push_back(std::get<1>(ac->forward(states_pos[c-1], states_map[c-1])));

                    returns = PPO::returns(rewards, dones, values, .99, .95);

                    torch::Tensor t_log_probs = torch::cat(log_probs).detach();
                    torch::Tensor t_returns = torch::cat(returns).detach();
                    torch::Tensor t_values = torch::cat(values).detach();
                    torch::Tensor t_states_pos = torch::cat(states_pos);
                    torch::Tensor t_states_map = torch::cat(states_map);
                    torch::Tensor t_actions = torch::cat(actions);
                    torch::Tensor t_advantages = t_returns - t_values.slice(0, 0, n_steps);

                    PPO::update(ac, t_states_pos, t_states_map, t_actions, t_log_probs, t_returns, t_advantages, opt, n_steps, ppo_epochs, mini_batch_size, beta);
                
                    c = 0;

                    states_pos.clear();
                    states_map.clear();
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
                torch::save(ac, "../../out/ppo_nmpc/ppo_nmpc_best_model.pt");
            }

            printf("Average reward: %f at entropy %f\n", avg_reward, avg_entropy);
        }
        error = false;
        avg_reward = 0.;
        avg_entropy = 0.;

        // Reset the environment.
        env.Reset();

        // episode, agent_x, agent_y, goal_x, goal_y, STATUS=(PLAYING, REACHEDGOAL, MISSEDGOAL, HITOBSTACLE, RESETTING)
        out << e+1 << ", " << env.pos_(0) << ", " << env.pos_(1) << ", " << env.obs_(0) << ", " << env.obs_(1) << ", " << env.r_obs_ << ", " << env.goal_(0) << ", " << env.goal_(1)  << ", " << RESETTING << "\n";
    }

    out_loss.close();
    out.close();

    return 0;
}

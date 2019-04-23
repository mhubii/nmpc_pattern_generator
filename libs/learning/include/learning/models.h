#pragma once

#include <torch/torch.h>
#include <math.h>

// Network model for Proximal Policy Optimization.
struct ActorCriticImpl : public torch::nn::Module 
{
    // Actor.
    torch::nn::Linear a_lin1_, a_lin2_, a_lin3_;
    torch::Tensor mu_;
    torch::Tensor log_std_;

    // Critic.
    torch::nn::Linear c_lin1_, c_lin2_, c_lin3_, c_val_;

    ActorCriticImpl(int64_t n_in, int64_t n_out, double std)
        : // Actor.
          a_lin1_(torch::nn::Linear(n_in, 16)),
          a_lin2_(torch::nn::Linear(16, 32)),
          a_lin3_(torch::nn::Linear(32, n_out)),
          mu_(torch::full(n_out, 0.)),
          log_std_(torch::full(n_out, std)),
          
          // Critic
          c_lin1_(torch::nn::Linear(n_in, 16)),
          c_lin2_(torch::nn::Linear(16, 32)),
          c_lin3_(torch::nn::Linear(32, n_out)),
          c_val_(torch::nn::Linear(n_out, 1)) 
    {
        // Register the modules.
        register_module("a_lin1", a_lin1_);
        register_module("a_lin2", a_lin2_);
        register_module("a_lin3", a_lin3_);
        register_parameter("log_std", log_std_);

        register_module("c_lin1", c_lin1_);
        register_module("c_lin2", c_lin2_);
        register_module("c_lin3", c_lin3_);
        register_module("c_val", c_val_);
    }

    // Forward pass.
    auto forward(torch::Tensor x) -> std::tuple<torch::Tensor, torch::Tensor> 
    {

        // Actor.
        mu_ = torch::relu(a_lin1_->forward(x));
        mu_ = torch::relu(a_lin2_->forward(mu_));
        mu_ = torch::tanh(a_lin3_->forward(mu_));

        // Critic.
        torch::Tensor val = torch::relu(c_lin1_->forward(x));
        val = torch::relu(c_lin2_->forward(val));
        val = torch::tanh(c_lin3_->forward(val));
        val = c_val_->forward(val);

        if (this->is_training()) 
        {
            torch::NoGradGuard no_grad;

            torch::Tensor action = torch::normal(mu_, log_std_.exp().expand_as(mu_));
            return std::make_tuple(action, val);  
        }
        else 
        {
            return std::make_tuple(mu_, val);  
        }
    }

    // Initialize network.
    void normal(double mu, double std) 
    {
        torch::NoGradGuard no_grad;

        for (auto& p: this->parameters()) 
        {
            p.normal_(mu,std);
        }         
    }

    auto entropy() -> torch::Tensor
    {
        // Differential entropy of normal distribution. For reference https://pytorch.org/docs/stable/_modules/torch/distributions/normal.html#Normal
        return 0.5 + 0.5*log(2*M_PI) + log_std_;
    }

    auto log_prob(torch::Tensor action) -> torch::Tensor
    {
        // Logarithmic probability of taken action, given the current distribution.
        torch::Tensor var = (log_std_+log_std_).exp();

        return -((action - mu_)*(action - mu_))/(2*var) - log_std_ - log(sqrt(2*M_PI));
    }
};

TORCH_MODULE(ActorCritic);

// Network model for Proximal Policy Optimization with Nonlinear Model Predictive Control.
struct ActorCriticNMPCImpl : public torch::nn::Module 
{
    // Actor.
    torch::nn::Linear a_lin1_, a_lin2_, a_lin3_;
    torch::nn::Conv2d a_conv1_, a_conv2_;
    torch::Tensor mu_;
    torch::Tensor log_std_;
    double mu_max_;
    double std_max_;

    // Critic.
    torch::nn::Linear c_lin1_, c_lin2_, c_lin3_, c_val_;
    torch::nn::Conv2d c_conv1_, c_conv2_;

    ActorCriticNMPCImpl(int64_t n_in, int64_t height, int64_t width, int64_t n_out, double mu_max, double std_max)
        : // Actor.
          a_lin1_(torch::nn::Linear(n_in, 16)),
          a_lin2_(torch::nn::Linear(16, 16)),
          a_conv1_(torch::nn::Conv2dOptions(1, 32, 5)),
          a_conv2_(torch::nn::Conv2dOptions(32, 32, 5)),
          a_lin3_(torch::nn::Linear(GetConvOutput(height, width)+16, n_out)),
          mu_(torch::full(n_out, 0.)),
          log_std_(torch::full(n_out, log(std_max), torch::kFloat64)),
          mu_max_(mu_max),
          std_max_(std_max),
          
          // Critic
          c_lin1_(torch::nn::Linear(n_in, 16)),
          c_lin2_(torch::nn::Linear(16, 16)),
          c_conv1_(torch::nn::Conv2dOptions(1, 32, 5)),
          c_conv2_(torch::nn::Conv2dOptions(32, 32, 5)),
          c_lin3_(torch::nn::Linear(GetConvOutput(height, width)+16, n_out)),
          c_val_(torch::nn::Linear(n_out, 1)) 
    {
        // Register the modules.
        register_module("a_lin1", a_lin1_);
        register_module("a_lin2", a_lin2_);
        register_module("a_lin3", a_lin3_);
        register_module("a_conv1", a_conv1_);
        register_module("a_conv2", a_conv2_);
        register_parameter("log_std", log_std_);

        register_module("c_lin1", c_lin1_);
        register_module("c_lin2", c_lin2_);
        register_module("c_lin3", c_lin3_);
        register_module("c_conv1", c_conv1_);
        register_module("c_conv2", c_conv2_);
        register_module("c_val", c_val_);
    }

    // Forward pass.
    auto forward(torch::Tensor pos, torch::Tensor map) -> std::tuple<torch::Tensor, torch::Tensor> 
    {

        // Actor.
        torch::Tensor a_fc_out = torch::relu(a_lin1_->forward(pos));
        a_fc_out = torch::relu(a_lin2_->forward(a_fc_out));

        torch::Tensor a_conv_out = torch::relu(a_conv1_->forward(map));
        a_conv_out = torch::relu(a_conv2_->forward(a_conv_out));
        
        // Flatten the output.
        a_conv_out = a_conv_out.view({a_conv_out.sizes()[0], -1});  

        // Concatenate the output.
        mu_ = torch::cat({a_fc_out, a_conv_out}, 0);
        
        mu_ = torch::tanh(a_lin3_->forward(mu_)).mul(mu_max_);

        // Critic.
        torch::Tensor c_fc_out = torch::relu(c_lin1_->forward(pos));
        c_fc_out = torch::relu(c_lin2_->forward(c_fc_out));

        torch::Tensor c_conv_out = torch::relu(c_conv1_->forward(map));
        c_conv_out = torch::relu(c_conv2_->forward(c_conv_out));
        
        // Flatten the output.
        c_conv_out = c_conv_out.view({c_conv_out.sizes()[0], -1});  


        torch::Tensor val = torch::cat({c_fc_out, c_conv_out}, 0);
        val = torch::tanh(c_lin3_->forward(val)).mul(mu_max_);
        val = c_val_->forward(val);

        // Reparametrization trick.
        if (this->is_training()) 
        {
            torch::NoGradGuard no_grad;

            torch::Tensor action = torch::normal(mu_, log_std_.exp().expand_as(mu_).mul(std_max_));
            return std::make_tuple(action, val);  
        }
        else 
        {
            return std::make_tuple(mu_, val);  
        }
    }

    // Get number of elements of output.
    int64_t GetConvOutput(int64_t height, int64_t width) {

        torch::Tensor in = torch::zeros({height, width}, torch::kF64).unsqueeze(0);
        torch::Tensor out = a_conv1_->forward(in);
        out = a_conv2_->forward(out);

        return out.numel();
    }

    // Initialize network.
    void normal(double mu, double std) 
    {
        torch::NoGradGuard no_grad;

        for (auto& p: this->parameters()) 
        {
            p.normal_(mu,std);
        }         
    }

    void zero() 
    {
        torch::NoGradGuard no_grad;

        for (auto& p: this->parameters()) 
        {
            p.zero_();
        }         
    }

    auto entropy() -> torch::Tensor
    {
        // Differential entropy of normal distribution. For reference https://pytorch.org/docs/stable/_modules/torch/distributions/normal.html#Normal
        return 0.5 + 0.5*log(2*M_PI) + log_std_;
    }

    auto log_prob(torch::Tensor action) -> torch::Tensor
    {
        // Logarithmic probability of taken action, given the current distribution.
        torch::Tensor var = (log_std_+log_std_).exp();

        return -((action - mu_)*(action - mu_))/(2*var) - log_std_ - log(sqrt(2*M_PI));
    }
};

TORCH_MODULE(ActorCriticNMPC);


// Model for learning the preview horizon of nonlinear model predictive control.
struct NmpcNetImpl : public torch::nn::Module
{
    // Preview horizon.
    torch::nn::Linear lin1_, lin2_, lin3_, lin4_, lin5_, lin6_, lin7_, lin8_, lin9_, lin10_, lin11_;
    // torch::nn::BatchNorm bn1_, bn2_, bn3_;

    NmpcNetImpl(int64_t n_in, int64_t n_out)
        : lin1_(n_in, 32),
          lin2_(32, 32),
          lin3_(32, 32),
          lin4_(32, 32),
          lin5_(32, 32),
          lin6_(32, 32),
          lin7_(32, 32),
          lin8_(32, 32),
          lin9_(32, 32),
          lin10_(32, 32),
          lin11_(32, n_out)
    {
        register_module("lin1", lin1_);
        register_module("lin2", lin2_);
        register_module("lin3", lin3_);
        register_module("lin4", lin4_);
        register_module("lin5", lin5_);
        register_module("lin6", lin6_);
        register_module("lin7", lin7_);
        register_module("lin8", lin8_);
        register_module("lin9", lin9_);
        register_module("lin10", lin10_);
        register_module("lin11", lin11_);
    }

    auto forward(torch::Tensor x) -> torch::Tensor
    {
        x = torch::tanh(lin1_(x));
        x = torch::tanh(lin2_(x));
        x = torch::tanh(lin3_(x));
        x = torch::tanh(lin4_(x));
        x = torch::tanh(lin5_(x));
        x = torch::tanh(lin6_(x));
        x = torch::tanh(lin7_(x));
        x = torch::tanh(lin8_(x));
        x = torch::tanh(lin9_(x));
        x = torch::tanh(lin10_(x));
        
        return lin11_(x);
    }

    auto normal(float mu, float std) -> void
    {
        torch::NoGradGuard no_grad;

        for (auto& p : this->parameters())
        {
            p.normal_(mu, std);
        }
    }
};

TORCH_MODULE(NmpcNet);

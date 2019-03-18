#pragma once

#include <torch/torch.h>
#include <math.h>

// Network model for Proximal Policy Optimization with Nonlinear Model Predictive Control.
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
          log_std_(torch::full(n_out, std, torch::kFloat64)),
          
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

        // Reparametrization trick.
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

TORCH_MODULE(ActorCritic);


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

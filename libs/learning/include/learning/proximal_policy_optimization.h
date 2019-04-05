#pragma once

#include <torch/torch.h>

#include "models.h"

// Vector of tensors.
using VT = std::vector<torch::Tensor>;

// Optimizer.
using OPT = torch::optim::Optimizer;

// Random engine for shuffling memory.
std::random_device rd;
std::mt19937 re(rd());

// Proximal policy optimization, https://arxiv.org/abs/1707.06347
class PPO
{
public:
    static auto returns(VT& rewards, VT& dones, VT& vals, double gamma, double lambda) -> VT; // Generalized advantage estimate, https://arxiv.org/abs/1506.02438
    static auto update(ActorCritic& ac,
                       torch::Tensor& states,
                       torch::Tensor& actions,
                       torch::Tensor& log_probs,
                       torch::Tensor& returns,
                       torch::Tensor& advantages, 
                       OPT& opt, 
                       uint steps, uint epochs, uint mini_batch_size, double beta, double clip_param=.2) -> void;
};


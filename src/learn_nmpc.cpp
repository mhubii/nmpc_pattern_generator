#include <torch/torch.h>

#include "base_generator.h"
#include "utils.h"

int64_t epochs = 10;
int64_t batch_size = 64;
int64_t log_interval = 10;

// Model for learning the preview horizon.
struct NetImpl : public torch::nn::Module
{
    // Preview horizon.
    torch::nn::Linear lin1_, lin2_, lin3_, lin4_;

    NetImpl(int64_t n_in, int64_t n_out)
        : lin1_(n_in, 64),
          lin2_(64, 128),
          lin3_(128, 64),
          lin4_(64, n_out)
    {
        register_module("lin1", lin1_);
        register_module("lin2", lin2_);
        register_module("lin3", lin3_);
        register_module("lin4", lin4_);
    }

    auto forward(torch::Tensor x) -> torch::Tensor
    {
        x = torch::relu(lin1_(x));
        x = torch::relu(lin2_(x));
        x = torch::relu(lin3_(x));
        
        return lin4_(x);
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

TORCH_MODULE(Net);


// Dataset that loads from .csv files.
namespace torch {
namespace data {
namespace datasets {

Tensor read_csv(const std::string& root, const std::string& csv /*e.g. ini, com, fkq, fkp*/)
{
    RowMatrixXd epochs = ReadCsv<RowMatrixXd>(root + "successful_epochs.csv");
    RowMatrixXd mat    = ReadCsv<RowMatrixXd>(root + csv + "_epoch_" + std::to_string(int(epochs(0))) + ".csv");

    auto tensor = torch::empty({epochs.size()*mat.rows(), 1, mat.cols()}, kF64);

    for (int i = 0; i < epochs.size(); i++) {
        mat = ReadCsv<RowMatrixXd>(root + csv + "_epoch_" + std::to_string(int(epochs(i))) + ".csv");

        for (int j = 0; j < mat.rows(); j++) {

            std::memcpy(tensor[i*mat.rows()+j].data_ptr(), mat.data()+j*mat.cols(), mat.cols()*sizeof(double));
        }
    }

    return tensor.to(kF32);
};

// Dataset for the pattern generator.
class PG : public Dataset<PG>
{
    private:
        Tensor states_, targets_;

    public:
        explicit PG(const std::string& root, 
                    const std::string& states,
                    const std::string& targets) 
            : states_(read_csv(root, states)),
              targets_(read_csv(root, targets)) 
            { 
                std::cout << "Dataset -- states of size: " << states_.sizes() << " loaded." << std::endl;
            };

        Example<> get(size_t index) override;

        optional<size_t> size() const override;
};

Example<> PG::get(size_t index)
{
    return {states_[index], targets_[index]};
} 

optional<size_t> PG::size() const
{
    return states_.size(0);
} 

} // namesapce datasets
} // namespace data
} // namespace torch


// Training functions.
template <typename DataLoader>
void train(int32_t epoch,
           Net& model,
           torch::Device device,
           DataLoader& data_loader,
           torch::optim::Optimizer& optimizer,
           size_t dataset_size) 
{
    model->train();
    size_t batch_idx = 0;
    for (auto& batch : data_loader) 
    {
        auto data = batch.data.to(device), targets = batch.target.to(device);
        optimizer.zero_grad();
        auto output = model->forward(data);
        auto loss = torch::mse_loss(output, targets);
        AT_ASSERT(!std::isnan(loss.template item<float>())); // thats nice!!!
        loss.backward();
        optimizer.step();

        if (batch_idx++ % log_interval == 0) 
        {
            std::printf(
            "\rTrain Epoch: %ld [%5ld/%5ld] Loss: %.4f",
            epoch,
            batch_idx * batch.data.size(0),
            dataset_size,
            loss.template item<float>());
        }
    }
}


int main() {

    // Load the dataset.
    std::string root = "data/";
    std::string states = "ini";
    std::string targets = "com";

    // Generate a dataset.
    auto data_set = torch::data::datasets::PG(root, states, targets).map(torch::data::transforms::Stack<>());

    int data_set_size = data_set.size().value();

    const int64_t batches_per_epoch =
        std::ceil(data_set.size().value() / static_cast<double>(batch_size));

    auto data_loader = torch::data::make_data_loader<torch::data::samplers::SequentialSampler>(
        std::move(data_set), 
        torch::data::DataLoaderOptions().batch_size(batch_size).workers(2));

    // Model and optimizer.
    torch::Device device(torch::kCUDA);

    int64_t n_in = 14;
    int64_t n_out = 32;

    printf("Initializing model.\n");
    Net model(n_in, n_out);
    model->to(torch::kF32);
    model->normal(0., 1.);
    model->to(device);
    torch::optim::Adam opt(model->parameters(), torch::optim::AdamOptions(1e-3));

    printf("Starting the training.\n");
    for (int epoch = 1; epoch <= epochs; epoch++)
    {
        train(epoch, model, device, *data_loader, opt, data_set_size);
    }

    return 0;
}


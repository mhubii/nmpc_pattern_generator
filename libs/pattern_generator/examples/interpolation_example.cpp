#include "base_generator.h"
#include "interpolate.h"

int main() {
    const std::string config_file = "../libs/pattern_generator/configs.yaml";
    BaseGenerator base_gen(config_file);
    TESTInterpolation inter(base_gen);

    inter.TESTInterpolate();
}
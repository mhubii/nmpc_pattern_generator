#include <iostream>

#include "base_generator.h"
#include "interpolate.h"
#include "interpolation.h"

int main() {
    // Base generator.
    const std::string config_file = "../libs/pattern_generator/configs.yaml";
    BaseGenerator base_gen(config_file);

    // Old implementation.
    Interpolation inter(0.005, base_gen);
    inter.Interpolate(0.);

    // New implementation.
    TESTInterpolation TESTinter(base_gen);
    TESTinter.TESTInterpolate();
}
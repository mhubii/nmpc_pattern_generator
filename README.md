# Nonlinear Model Predictive Control for Walking Pattern Generation
This project implements [A Reactive Walking Pattern Generator Based on Nonlinear Model Predictive Control](https://hal.archives-ouvertes.fr/hal-01261415/document) to generate stable walking trajectories for a humanoid robot.

<br>
<figure>
  <p align="center"><img src="img/heicub_walking.gif" width="80%" height="80%"></p>
  <figcpation>Fig. 1: Visualization of generated walking trajectories with <a href="https://github.com/ORB-HD/MeshUp">MeshUp</a>.</figcaption>
</figure>
<br><br>

## Build

### Dependencies

## Usage
An example on how the NMPC pattern generator is ment to be used, can be executed by calling
```
cd build/bin
./nmpc_generator_example
```
The generated center of mass and feet trajectories are then written to `build/bin/example_nmpc_generator_interpolated_results.csv`. They can be visualized by 

```
cd plot
python plot_pattern.py
```

<br>
<figure>
  <p align="center"><img src="img/generated_nmpc_pattern.png" width="60%" height="60%"></p>
  <figcpation>Fig. 2: Generated center of mass and feet trajectories.</figcaption>
</figure>
<br><br>

We will go through the most important parts of the pattern generation in the following


```cpp
// Initialize pattern generator.
const std::string config_file_loc = "../../libs/pattern_generator/configs.yaml";

NMPCGenerator nmpc(config_file_loc);

// Pattern generator preparation.
nmpc.SetSecurityMargin(nmpc.SecurityMarginX(), 
                       nmpc.SecurityMarginY());

// Set initial values.
PatternGeneratorState pg_state = {nmpc.Ckx0(),
                                  nmpc.Cky0(),
                                  nmpc.Hcom(),
                                  nmpc.Fkx0(),
                                  nmpc.Fky0(),
                                  nmpc.Fkq0(),
                                  nmpc.CurrentSupport().foot,
                                  nmpc.Ckq0()};

nmpc.SetInitialValues(pg_state);
Interpolation interpol_nmpc(nmpc);
interpol_nmpc.StoreTrajectories(true);
Eigen::Vector3d velocity_reference(0., 0., 0.1);

// Pattern generator event loop.
for (int i = 0; i < 100; i++) {
    std::cout << "Iteration: " << i << std::endl;

    // Set reference velocities.
    nmpc.SetVelocityReference(velocity_reference);

    // Solve QP.
    nmpc.Solve();
    nmpc.Simulate();
    interpol_nmpc.InterpolateStep();

    // Initial value embedding by internal states and simulation.
    pg_state = nmpc.Update();
    nmpc.SetInitialValues(pg_state);
}

// Save interpolated results.
Eigen::MatrixXd trajectories = interpol_nmpc.GetTrajectories().transpose();
WriteCsv("example_nmpc_generator_interpolated_results.csv", trajectories);
```


## Run Tests
To verify your installation, you can run the provided tests

```
cd build/bin
./pattern_generator_tests
```

The tests are written with [googletest](https://github.com/google/googletest), which is included as a submodule. They should output

```
[==========] Running 4 tests from 3 test suites.
[----------] Global test environment set-up.
[----------] 2 tests from CompareMPCToNMPC
[ RUN      ] CompareMPCToNMPC.Submatrices
[       OK ] CompareMPCToNMPC.Submatrices (49 ms)
[ RUN      ] CompareMPCToNMPC.CompareConstraintMatrices
[       OK ] CompareMPCToNMPC.CompareConstraintMatrices (26 ms)
[----------] 2 tests from CompareMPCToNMPC (75 ms total)

[----------] 1 test from MPCGeneratorTest
[ RUN      ] MPCGeneratorTest.Solve
[       OK ] MPCGeneratorTest.Solve (69 ms)
[----------] 1 test from MPCGeneratorTest (69 ms total)

[----------] 1 test from NMPCGeneratorTest
[ RUN      ] NMPCGeneratorTest.Solve
[       OK ] NMPCGeneratorTest.Solve (225 ms)
[----------] 1 test from NMPCGeneratorTest (225 ms total)

[----------] Global test environment tear-down
[==========] 4 tests from 3 test suites ran. (369 ms total)
[  PASSED  ] 4 tests.
```

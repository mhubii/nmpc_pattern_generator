# Configurations.
cmake_minimum_required(VERSION 3.8)

set(CMAKE_INSTALL_PREFIX $ENV{HOME}/nmpc_pattern_generator CACHE STRING "Install path prefix default.")

project(heicub_walking)

set(CMAKE_CXX_STANDARD 17)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# Google test library.
# Download and unpack googletest at configure time.
enable_testing()

configure_file(cmake/CMakeLists.txt.in_googletest googletest-download/CMakeLists.txt)

execute_process(COMMAND "${CMAKE_COMMAND}" -G "${CMAKE_GENERATOR}" .
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/googletest-download" )

execute_process(COMMAND "${CMAKE_COMMAND}" --build .
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/googletest-download" )


# Prevent googletest from overriding our compiler/linker options
# when building with Visual Studio.
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)


# Add googletest directly to our build. This adds
# the following targets: gtest, gtest_main, gmock
# and gmock_main.
add_subdirectory("${CMAKE_BINARY_DIR}/googletest-src"
                 "${CMAKE_BINARY_DIR}/googletest-build")

# Output for all libs.
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# Input output module library.
option(IO_MODULE_EXAMPLES "Build input output module examples." ON)

option(IO_MODULE_TESTS "Build input output module tests." ON)

add_subdirectory(libs/io_module)


# Kinematics library.
option(KINEMATICS_EXAMPLES "Build kinematics examples." ON)

option(KINEMATICS_TESTS "Build kinematics tests." ON)

add_subdirectory(libs/kinematics)


# Pattern generator library.
option(PATTERN_GENERATOR_EXAMPLES "Build pattern generator examples." ON)

option(PATTERN_GENERATOR_TESTS "Build pattern generator tests." ON)

add_subdirectory(libs/pattern_generator)


# Reinforcement learning library.
option(REINFORCEMENT_LEARNING_EXAMPLES "Build reinforcement learning examples." ON)

option(REINFORCEMENT_LEARNING_TESTS "Build reinforcement learning tests." ON)

add_subdirectory(libs/reinforcement_learning)


# Source.
set(SOURCES
    app_user_interface
    artificially_controlled_walking
    keyboard_user_interface
    # offline_walking
    ppo_nmpc
    test_pg
    test_robot
    # train_artificial_walking
    user_controlled_walking
)

foreach (source ${SOURCES})
    add_executable(${source}
        src/${source}.cpp
    )

    target_include_directories(${source}
        PRIVATE libs/io_module/include/io_module
        PRIVATE libs/kinematics/include/kinematics
        PRIVATE libs/pattern_generator/include/pattern_generator
        PRIVATE libs/reinforcement_learning/include/reinforcement_learning
    )

    target_link_libraries(${source}
        io_module
        kinematics
        pattern_generator
        reinforcement_learning
    )
endforeach(source ${SOURCES})

# Uninstall directives.
add_custom_target (uninstall COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/nmpc_pattern_generator_uninstall.cmake)

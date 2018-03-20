# Configurations.
cmake_minimum_required(VERSION 3.8)

project(heicub_walking)

set(CMAKE_CXX_STANDARD 17)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)


# Dynamic filter library.
# add_subdirectory(dynamic_filter)


# Google test library.
# Download and unpack googletest at configure time.
enable_testing()

configure_file(CMakeLists.txt.in googletest-download/CMakeLists.txt)

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


# Input output library.
# add_subdirectory(input_output)


# Inverse kinematics library.
# add_subdirectory(inverse_kinematics)


# Pattern generator library.
option(PATTERN_GENERATOR_EXAMPLES "Build pattern generator examples." ON)

option(PATTERN_GENERATOR_TESTS "Build pattern generator tests." ON)

add_subdirectory(libs/pattern_generator)


# Reinforced walking library.
# add_subdirectory(reinforced_walking)


# Source.

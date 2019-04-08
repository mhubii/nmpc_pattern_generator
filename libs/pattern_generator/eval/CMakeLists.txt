cmake_minimum_required(VERSION 3.11 FATAL_ERROR)

set(KINEMATICS_DIR $ENV{HOME}/nmpc_pattern_generator CACHE STRING "Directory of kinematics.")
set(PATTERN_GENERATOR_DIR $ENV{HOME}/nmpc_pattern_generator CACHE STRING "Directory of pattern_generator.")

project(pg_eval)

set(CMAKE_CXX_STANDARD 17)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(kinematics REQUIRED)
find_package(pattern_generator REQUIRED)
find_package(qpOASES REQUIRED)

include_directories(${KINEMATICS_INCLUDE_DIR})
include_directories(${PATTERN_GENERATOR_INCLUDE_DIR})

add_executable(generate_steps generate_steps.cpp)
target_link_libraries(generate_steps ${KINEMATICS_LIBRARY} ${PATTERN_GENERATOR_LIBRARY} ${qpOASES_LIBRARY})
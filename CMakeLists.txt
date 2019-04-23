# Configurations.
cmake_minimum_required(VERSION 3.10)

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
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/bin)

# Pattern generator library.
option(PATTERN_GENERATOR_EXAMPLES "Build pattern generator examples." ON)
option(PATTERN_GENERATOR_TESTS "Build pattern generator tests." ON)
add_subdirectory(libs/pattern_generator)

# Additional libraries to build.
option(BUILD_WITH_YARP "Build the libraries for the real robot and gazebo." OFF)
option(BUILD_WITH_LEARNING "Build the deep learning library." OFF)

# Libraries for controlling the robot in real and simulation.
if (${BUILD_WITH_YARP})
    # Input output module library.
    option(IO_MODULE_EXAMPLES "Build input output module examples." ON)
    add_subdirectory(libs/io_module)

    # Kinematics library.
    option(KINEMATICS_EXAMPLES "Build kinematics examples." ON)
    add_subdirectory(libs/kinematics)

    # Source.
    set(SOURCES
        app_user_interface
        keyboard_user_interface
        offline_walking
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
        )

        target_link_libraries(${source}
            io_module
            kinematics
            pattern_generator
        )
    endforeach(source ${SOURCES})
endif (${BUILD_WITH_YARP})

# Reinforcement learning library.
if (${BUILD_WITH_LEARNING})
    option(LEARNING_EXAMPLES "Build reinforcement learning examples." ON)
    add_subdirectory(libs/learning)

    # Source.
#    set(SOURCES 
#        learn_nmpc_fc
#        ppo_nmpc
#        predict_nmpc
#    )

#    foreach (source ${SOURCES})
#        add_executable(${source}
#            src/${source}.cpp
#        )

#        target_include_directories(${source}
#            PRIVATE libs/pattern_generator/include/pattern_generator
#            PRIVATE libs/learning/include/learning
#        )

#        target_link_libraries(${source}
#            pattern_generator
#            learning
#        )
#    endforeach(source ${SOURCES})
endif(${BUILD_WITH_LEARNING})

# Autonomous navigation sources.
if (${BUILD_WITH_YARP} AND ${BUILD_WITH_LEARNING})
    
    option(BUILD_WITH_OPENCV_CONTRIB "Whether to build with contribution modules of OpenCV" OFF)

    #find_package(Torch REQUIRED)
    find_package(OpenCV REQUIRED)

    # Source.
    set(SOURCES
        behavioural_cloning
        behavioural_augmentation
        ppo_nmpc        
    )

    foreach (source ${SOURCES})
        add_executable(${source}
            src/${source}.cpp
        )

        target_include_directories(${source}
            PRIVATE libs/io_module/include/io_module
            PRIVATE libs/kinematics/include/kinematics
            PRIVATE libs/pattern_generator/include/pattern_generator
            PRIVATE libs/learning/include/learning
            #PRIVATE ${TORCH_INCLUDE_DIRS}
            PRIVATE ${OpenCV_INCLUDE_DIRS}
        )

        target_link_libraries(${source}
            io_module
            kinematics
            pattern_generator
            learning
            #${TORCH_LIBRARIES}
            ${OpenCV_LIBRARIES}
        )
    endforeach(source ${SOURCES})
endif (${BUILD_WITH_YARP} AND ${BUILD_WITH_LEARNING})

# Uninstall directives.
add_custom_target (uninstall COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/nmpc_pattern_generator_uninstall.cmake)

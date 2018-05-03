# Add kinematics library.
find_package(yaml-cpp REQUIRED)
find_package(rbdl REQUIRED)
find_package(rbdl_urdfreader REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/kinematics)

set(SOURCE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/inverse_kinematics.cpp
)

add_library(kinematics SHARED
    ${SOURCE}
)

target_link_libraries(kinematics
    yaml-cpp
    ${rbdl_LIBRARIES}
    rbdl_urdfreader
)


# Build examples.
if (${KINEMATICS_EXAMPLES})
    set(EXAMPLES
        test_inverse_kinematics
    )

    foreach (example ${EXAMPLES})
        add_executable(${example}
            src/${example}.cpp
        )

        target_link_libraries(${example}
            kinematics
            pattern_generator
        )
    endforeach(example ${EXAMPLES})
endif (${KINEMATICS_EXAMPLES})


# Build tests.
if (${KINEMATICS_TESTS})

endif (${KINEMATICS_TESTS})

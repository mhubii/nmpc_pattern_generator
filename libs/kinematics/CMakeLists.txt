# Add kinematics library.
find_package(rbdl REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/kinematics)

set(SOURCE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/inverse_kinematics.cpp
)

add_library(kinematics SHARED
    ${SOURCE}
)

target_link_libraries(kinematics
    ${rbdl_LIBRARIES}
)


# Build examples.
if (${KINEMATICS_EXAMPLES})
    set(EXAMPLES

    )

    foreach (example ${EXAMPLES})
        add_executable(${example}
            src/${example}.cpp
        )

        target_link_libraries(${example}

        )
    endforeach(example ${EXAMPLES})
endif (${KINEMATICS_EXAMPLES})


# Build tests.
if (${KINEMATICS_TESTS})

endif (${KINEMATICS_TESTS})

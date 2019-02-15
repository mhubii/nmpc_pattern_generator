# Add kinematics library.
find_package(yaml-cpp REQUIRED)
find_package(rbdl REQUIRED)
find_package(rbdl_urdfreader REQUIRED)

set(KINEMATICS_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include/kinematics)
include_directories(${KINEMATICS_INCLUDE_DIR})

# Headers for installation.
list(APPEND KINEMATICS_INCLUDES ${KINEMATICS_INCLUDE_DIR}/kinematics.h
                                ${KINEMATICS_INCLUDE_DIR}/utils.h)

set(SOURCE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/kinematics.cpp
)

add_library(kinematics SHARED
    ${SOURCE}
)

target_link_libraries(kinematics
    yaml-cpp
    ${rbdl_LIBRARY}
    ${rbdl_urdfreader_LIBRARY}
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

# Install directives kinematics.
install(TARGETS kinematics DESTINATION lib)
install(FILES ${KINEMATICS_INCLUDES} DESTINATION include/kinematics)

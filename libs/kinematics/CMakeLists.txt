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
        inverse_kinematics_example
    )

    foreach (example ${EXAMPLES})
        add_executable(${example}
            examples/${example}.cpp
        )

        target_link_libraries(${example}
            kinematics
        )
    endforeach(example ${EXAMPLES})
endif (${KINEMATICS_EXAMPLES})


# Install directives kinematics.
install(TARGETS kinematics DESTINATION lib)
install(FILES ${KINEMATICS_INCLUDES} DESTINATION include/kinematics)

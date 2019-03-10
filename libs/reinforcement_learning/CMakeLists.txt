# Add reinforcement learning library.
find_package(Torch REQUIRED)

set(REINFORCEMENT_LEARNING_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include/reinforcement_learning)
include_directories(${REINFORCEMENT_LEARNING_INCLUDE_DIR})

# Headers for installation.
list(APPEND REINFORCEMENT_LEARNING_INCLUDES ${REINFORCEMENT_LEARNING_INCLUDE_DIR}/proximal_policy_optimization.h)

set(SOURCE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/proximal_policy_optimization.cpp
)

add_library(reinforcement_learning SHARED
    ${SOURCE}
)

target_link_libraries(reinforcement_learning
    ${TORCH_LIBRARIES}
)


# Build examples.
if (${REINFORCEMENT_LEARNING_EXAMPLES})
    set(EXAMPLES

    )

    foreach (example ${EXAMPLES})
        add_executable(${example}
            src/${example}.cpp
        )

        target_link_libraries(${example}
            reinforcement_learning
        )
    endforeach(example ${EXAMPLES})
endif (${REINFORCEMENT_LEARNING_EXAMPLES})


# Build tests.
if (${REINFORCEMENT_LEARNING_TESTS})

endif (${REINFORCEMENT_LEARNING_TESTS})

# Install directives reinforcement learning.
install(TARGETS reinforcement_learning DESTINATION lib)
install(FILES ${REINFORCEMENT_LEARNING_INCLUDES} DESTINATION include/reinforcement_learning)

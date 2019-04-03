# Add reinforcement learning library.
find_package(Torch REQUIRED)

set(LEARNING_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include/learning)
include_directories(${LEARNING_INCLUDE_DIR})
include_directories(${TORCH_INCLUDE_DIRS})

# Headers for installation.
list(APPEND LEARNING_INCLUDES ${LEARNING_INCLUDE_DIR}/proximal_policy_optimization.h)

set(SOURCE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/proximal_policy_optimization.cpp
)

add_library(learning SHARED
    ${SOURCE}
)

target_link_libraries(learning
    ${TORCH_LIBRARIES}
)


# Build examples.
if (${LEARNING_EXAMPLES})
    set(EXAMPLES
        train_ppo_example
        test_ppo_example
    )

    foreach (example ${EXAMPLES})
        add_executable(${example}
            examples/${example}.cpp
        )

        target_link_libraries(${example}
            learning
            pattern_generator
        )
    endforeach(example ${EXAMPLES})
endif (${LEARNING_EXAMPLES})

# Install directives reinforcement learning.
install(TARGETS learning DESTINATION lib)
install(FILES ${LEARNING_INCLUDES} DESTINATION include/learning)

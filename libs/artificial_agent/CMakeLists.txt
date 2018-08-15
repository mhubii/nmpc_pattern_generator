# Add pattern generator library.
find_library(CAFFE2_LIB caffe2)
find_library(CAFFE2_GPU_LIB caffe2_gpu)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/artificial_agent)

set(SOURCE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/dqn.cpp
)

add_library(artificial_agent SHARED
    ${SOURCE} 
)

target_link_libraries(artificial_agent
    caffe2
)


# Build examples.
if (${ARTIFICIAL_AGENT_EXAMPLES})
    set(EXAMPLES

    )

    foreach (example ${EXAMPLES})
        add_executable(${example}
            examples/${example}.cpp
        )

        target_link_libraries(${example}
            pattern_generator
        )
    endforeach(example ${EXAMPLES})
endif (${ARTIFICIAL_AGENT_EXAMPLES})

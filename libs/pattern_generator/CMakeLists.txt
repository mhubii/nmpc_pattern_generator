# Add pattern generator library.
find_package(qpOASES REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/pattern_generator)

set(SOURCE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/base_generator.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/interpolate.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/interpolation.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/mpc_generator.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/nmpc_generator.cpp
)

add_library(pattern_generator SHARED
    ${SOURCE} 
)

target_link_libraries(pattern_generator
    qpOASES
    yaml-cpp
)


# Build examples.
if (${PATTERN_GENERATOR_EXAMPLES})
    set(EXAMPLES
        mpc_generator_example
        nmpc_generator_example
        interpolation_example
    )

    foreach (example ${EXAMPLES})
        add_executable(${example}
            examples/${example}.cpp
        )

        target_link_libraries(${example}
            pattern_generator
        )
    endforeach(example ${EXAMPLES})
endif (${PATTERN_GENERATOR_EXAMPLES})


# Build tests.
if (${PATTERN_GENERATOR_TESTS})
    add_executable(pattern_generator_tests
        tests/compare_mpc_to_nmpc.cpp
        tests/test_mpc_generator.cpp
        tests/test_nmpc_generator.cpp
    )

    target_link_libraries(pattern_generator_tests
        gtest
        gtest_main
        pattern_generator
    )

    add_test(pattern_generator_tests
        pattern_generator_tests
    )
endif(${PATTERN_GENERATOR_TESTS})

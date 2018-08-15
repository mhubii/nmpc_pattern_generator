# Add input ouput module library.
find_package(YARP REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Curses REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/io_module)

set(SOURCE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/reader.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/writer.cpp
)

add_library(io_module SHARED
    ${SOURCE}
)

target_link_libraries(io_module
    ${YARP_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${CURSES_LIBRARIES}
    yaml-cpp
)


# Build examples.
if (${IO_MODULE_EXAMPLES})
    set(EXAMPLES
        test_depth
        test_reader
        test_writer
    )

    foreach (example ${EXAMPLES})
        add_executable(${example}
            src/${example}.cpp
        )

        target_link_libraries(${example}
            io_module
        )
    endforeach(example ${EXAMPLES})
endif (${IO_MODULE_EXAMPLES})


# Build tests.
if (${IO_MODULE_TESTS})

endif (${IO_MODULE_TESTS})

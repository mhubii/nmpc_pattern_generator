# Add input ouput module library.
find_package(YARP REQUIRED)
find_package(Curses REQUIRED)
find_package(yaml-cpp REQUIRED)

set(IO_MODULE_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include/io_module)
include_directories(${IO_MODULE_INCLUDE_DIR})

# Headers for installation.
list(APPEND IO_MODULE_INCLUDES ${IO_MODULE_INCLUDE_DIR}/reader.h
                               ${IO_MODULE_INCLUDE_DIR}/writer.h
                               ${IO_MODULE_INCLUDE_DIR}/utils.h)

set(SOURCE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/reader.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/writer.cpp
)

add_library(io_module SHARED
    ${SOURCE}
)

target_link_libraries(io_module
    ${YARP_LIBRARIES}
    ${CURSES_LIBRARIES}
    yaml-cpp
)


# Build examples.
if (${IO_MODULE_EXAMPLES})
    set(EXAMPLES
        reader_example
	writer_example
    )

    foreach (example ${EXAMPLES})
        add_executable(${example}
            examples/${example}.cpp
        )

        target_link_libraries(${example}
            io_module
        )
    endforeach(example ${EXAMPLES})
endif (${IO_MODULE_EXAMPLES})


# Install directives input output module library.
install(TARGETS io_module DESTINATION lib)
install(FILES ${IO_MODULE_INCLUDES} DESTINATION include/io_module)

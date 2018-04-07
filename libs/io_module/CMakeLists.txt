# Add input ouput module library.

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/io_module)

set(SOURCE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/read.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/write.cpp
)

add_library(io_module SHARED
    ${SOURCE}
)

#target_link_libraries(io_module
#    ...
#)


# Build exampled.
if (${IO_MODULE_EXAMPLES})

endif (${IO_MODULE_EXAMPLES})


# Build tests.
if (${IO_MODULE_TESTS})

endif (${IO_MODULE_TESTS})

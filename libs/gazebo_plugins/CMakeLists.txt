# Gazebo plugins.
find_package(gazebo REQUIRED)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/gazebo_plugins ${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")


# Artificially controlled iCub plugin.
add_library(gazeboAICubPlugin SHARED src/AICubPlugin.cpp)
target_link_libraries(gazeboAICubPlugin ${GAZEBO_LIBRARIES})

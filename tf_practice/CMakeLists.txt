cmake_minimum_required(VERSION 2.8.3)
project(tf_practice)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS roscpp tf)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    tf
)

catkin_package(
)

include_directories(
    include ${catkin_INCLUDE_DIRS}
)

add_executable(tf_practice
    src/tf_practice.cpp
)
add_dependencies(tf_practice ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tf_practice
    ${catkin_LIBRARIES}
)
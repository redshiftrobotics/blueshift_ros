cmake_minimum_required(VERSION 3.0.2)
project(robot_movements)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

generate_messages(DEPENDENCIES std_msgs)


catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})


#added executables
add_executable(testTalker src/testTalker.cpp)
target_link_libraries(testTalker ${catkin_LIBRARIES})
add_dependencies(testTalker beginner_tutorials_generate_messages_cpp)

add_executable(testListener src/testListener.cpp)
target_link_libraries(testListener ${catkin_LIBRARIES})
add_dependencies(testListener beginner_tutorials_generate_messages_cpp)
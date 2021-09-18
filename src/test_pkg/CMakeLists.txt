cmake_minimum_required(VERSION 3.0.2)
project(test_pkg)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(talker src/test_talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})

add_executable(listener src/test_listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
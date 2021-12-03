cmake_minimum_required(VERSION 3.5)
project(robot_core)


if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(talker rclcpp geometry_msgs)

install(TARGETS
  talker
  DESTINATION lib/${PROJECT_NAME})

ament_package()

add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp geometry_msgs)

install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME})

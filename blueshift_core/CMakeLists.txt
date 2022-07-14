cmake_minimum_required(VERSION 3.5)
project(blueshift_core)


if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)  
find_package(geometry_msgs REQUIRED)
find_package(blueshift_interfaces REQUIRED)   


add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp blueshift_interfaces) 

add_executable(control src/blueshift_driver.cpp src/holonomic/holonomic.cpp)
ament_target_dependencies(control rclcpp blueshift_interfaces geometry_msgs)

add_executable(hardware src/blueshift_hardware.cpp src/customi2clib/i2c.c)
ament_target_dependencies(hardware rclcpp blueshift_interfaces)


install(TARGETS
  listener
  control
  hardware
  DESTINATION lib/${PROJECT_NAME})


ament_package()




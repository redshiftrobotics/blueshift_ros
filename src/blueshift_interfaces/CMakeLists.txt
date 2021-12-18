cmake_minimum_required(VERSION 3.8)
project(blueshift_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/Time.msg"
  "msg/Num.msg"
)

set(srv_files
#  "srv/Service1.srv"
#  "srv/Service2.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
)

add_custom_command(
  OUTPUT
    ${CMAKE_SOURCE_DIR}/ts/ros_interfaces.ts
  COMMAND
    cd ${CMAKE_SOURCE_DIR} && /usr/bin/python3 util/generate_ts_interfaces.py ${CMAKE_BINARY_DIR} ${PROJECT_NAME} ${msg_files} ${srv_files}
  COMMENT
    "Generating typescript interface definitions"
  VERBATIM
)

add_custom_target(
  ts_interfaces ALL
  DEPENDS ${CMAKE_SOURCE_DIR}/ts/ros_interfaces.ts
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

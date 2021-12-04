cmake_minimum_required(VERSION 3.8)
project(blueshift_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosbridge_suite REQUIRED)

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

# build the node.js code before copying into install
add_custom_command(
  OUTPUT
    # This file doesn't exist, and it never should
    # We're tricking cmake into thinking that running the following commands will create this file
    # So if the file existed it wouldn't run the commands to build the website
    ${CMAKE_SOURCE_DIR}/build/build.stamp
  COMMAND
    echo "Installing all build dependencies" && cd ${CMAKE_SOURCE_DIR} && pnpm i # Install all dependencies
  COMMAND
    echo "Building the interface" && cd ${CMAKE_SOURCE_DIR} && pnpm run build # Build the site
  COMMENT
    "Building web interface with SvelteKit"
  VERBATIM
)

add_custom_target(
  web_interface ALL
  DEPENDS ${CMAKE_SOURCE_DIR}/build/index.js
)

# add_custom_target (npm-target ALL
#   COMMAND cd ${CMAKE_SOURCE_DIR} && npm run build
# )


# install rules added by ros2pkg_configure_nodejs
install(FILES 
 package.json
 DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  launch
  build
  node_modules
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
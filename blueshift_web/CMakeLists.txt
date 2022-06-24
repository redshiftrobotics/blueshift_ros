cmake_minimum_required(VERSION 3.8)
project(blueshift_web)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosbridge_suite REQUIRED)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # uncomment the line when a copyright and license is not present in all source files
#   #set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # uncomment the line when this package is not in a git repo
#   #set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

# build the node.js code before copying into install
# add_custom_command(
#   OUTPUT
#     # This file doesn't exist, and it never should
#     # We're tricking cmake into thinking that running the following commands will create this file
#     # So if the file existed it wouldn't run the commands to build the website
#     ${CMAKE_SOURCE_DIR}/build/build.stamp
#   COMMAND
#     echo "Installing all build dependencies" && cd ${CMAKE_SOURCE_DIR} && pnpm i # Install all dependencies
#   COMMAND
#     echo "Building the web interface" && cd ${CMAKE_SOURCE_DIR} && pnpm run build # Build the site
#   COMMAND
#     echo "Removing unnecessary libraries" && cd ${CMAKE_SOURCE_DIR} && rm -rf node_modules # delete the node modules folder so we don't copy it
#   COMMENT
#     "Building web interface with SvelteKit"
#   VERBATIM
# )

# add_custom_target(
#   web_interface ALL
#   DEPENDS ${CMAKE_SOURCE_DIR}/build/build.stamp
# )

# This is a simpler version of the add_custom_command and add_custom_target above
# add_custom_target (npm-target ALL
#   COMMAND cd ${CMAKE_SOURCE_DIR} && pnpm i && pnpm run build
# )

# We don't need to copy this over, but it doesn't hurt to have it there
install(FILES 
 package.json
 DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  launch
  build
  DESTINATION share/${PROJECT_NAME}/
#   # DO NOT copy node_modules. There are so many libraries that it will take 5+ minues to copy them over
#   # Additionally because svelte compiles all the libraries into your app, there are no runtime dependencies
#   PATTERN "*/node_modules/*" EXCLUDE
)

ament_package()
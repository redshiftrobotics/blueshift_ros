# blueshift-ros
## Setup Instructions
Install ROS Galactic following these instructions:
- https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html
- https://docs.ros.org/en/galactic/Tutorials/Configuring-ROS2-Environment.html
- https://docs.ros.org/en/galactic/Tutorials/Colcon-Tutorial.html

clone this respository

then run `rosdep install -i --from-path src --rosdistro galactic -y`

To build the whole project run `colcon build`
- if you get the error `AssertionError: No verb extentions` follow [these instructions](https://github.com/aws-robotics/aws-iot-bridge-example/issues/2#issuecomment-810040837)
To build specific packages run `colcon build --packages-select YOUR_PKG_NAME`
- `--event-handlers console_direct+` can be added to the end of any `colcon` command to enable verbose output
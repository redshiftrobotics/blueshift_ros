# blueshift-ros

## Setup Instructions

_Note: these instructions are meant for Ubuntu. If you are using a different distribution, ie windows or macOS, it will be more complicated._

Install ROS Galactic following these instructions:

- <https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html>
- <https://docs.ros.org/en/galactic/Tutorials/Configuring-ROS2-Environment.html>
- <https://docs.ros.org/en/galactic/Tutorials/Colcon-Tutorial.html>
  - After creating a workspace, skip down to [*Setup `colcon_cd`*](https://docs.ros.org/en/galactic/Tutorials/Colcon-Tutorial.html#setup-colcon-cd)

### Clone this repository into the `src` folder of your workspace

```bash
git clone https://github.com/redshiftrobotics/blueshift-ros.git
```

### Install `rosdep`
```bash
sudo apt install -y python3-rosdep
sudo rosdep init
rosdep update --include-eol-distros
```

### Install Dependencies

_Note_: All commands after this point must be run in the root folder your colcon workspace

```bash
rosdep install -i --from-path src --rosdistro galactic -y
```

## Build project

To build the whole project run `colcon build`.
Note that you will need to follow the installation instructions in [blueshift_web](./blueshift_web/README.md) before it will build.

- if you get the error `AssertionError: No verb extentions` follow [these instructions](https://github.com/aws-robotics/aws-iot-bridge-example/issues/2#issuecomment-810040837)
- To build specific packages run `colcon build --packages-select YOUR_PKG_NAME`
- `--event-handlers console_direct+` can be added to the end of any `colcon` command to enable verbose output

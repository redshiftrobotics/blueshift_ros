# Setup Instructions

## Install `node` 17

### `nvm` method (works on any OS, recommended for development on personal computers)
I would recommend using nvm [`nvm`](https://github.com/nvm-sh/nvm) (**N**ode **V**ersion **M**anager), it provides an easy way to install multiple versions of node on a single computer.

**Install `nvm`**
```bash
curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/master/install.sh | bash
```
Open a new terminal to get access to `nvm`

**Install `node`**
```bash
nvm install 17
```
### `snap` method (works on ubuntu only, recommended for the robot's computer)
```bash
sudo snap install node --classic --channel=17
```

### Verify that node is correctly installed and that you have the version
```bash
node -v
```
This should print out `v17.*.*` if node is installed correctly

## Install `pnpm`

We use, [`pnpm`](https://github.com/pnpm/pnpm) instead of [`npm`](https://www.npmjs.com/about). Read why [here](https://betterprogramming.pub/the-case-for-pnpm-over-npm-or-yarn-2b221607119)

_In general, `pnpm` is a drop-in replacement for `npm` but it is faster, uses less storage, and is generally better.
Any `npm` command should work the exact same if you just replace it with `pnpm`._

Install it with:

```bash
npm install -g pnpm
```

## Download all node packages
_Note_: All of the following commands including this one need to be run in the `blueshift_web` directory

```bash
pnpm install
```

## Run the website for development

_Note_: ~~Until RobotWebTools/roslibjs#548 is fixed, running the website in development mode won't work. The ROS based build/launch scripts are still functional, but when developing the website, it needs to be manually built using `pnpm run build` and then `pnpm run preview`~~ This should be fixed (see [this](https://github.com/redshiftrobotics/blueshift_ros/commit/4123aa6c83aa3b5ebce4e4faf1022b70786aecab) commit). Its a kind of janky hack, but works.

```bash
pnpm run dev
```

If it needs to be accessible on other computers on your network

```bash
pnpm run dev -- --host
```

If the website needs to communicate with ROS (i.e. you are doing anything more than UI development), you need to start the rosbridge node:
```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

## Running the website with ROS

To build it:

```bash
colcon build --packages-select blueshift_web
```

~~_Note 1_: This first time it is built, this might take a long time, because all the node packages need to be copied over to the install folder. Every subsequent time, it should only copy over the difference, so it should be faster. _TODO later: there may be a way to just re-download them into the install directory which might be quicker._~~

~~_Note 2_: Building this with the `--symlink-install` flag seems to raise cause an issue with symlinks, so this package (and probably the whole workspace) should be built _without_ that flag.~~ These should be fixed as [of](https://github.com/redshiftrobotics/blueshift_ros/commit/8627c5a2cc00898b1daa676ac3e7ce17158a3080) [these](https://github.com/redshiftrobotics/blueshift_ros/commit/2983a4e9bd095231ad10006e8be98e7f1cd1f222) [commits](https://github.com/redshiftrobotics/blueshift_ros/commit/8b9004af5f8bbead86375f798ccee82f673eed8b)

To run it (this also starts the rosbridge_server to facilitate JS<->ROS communication)

```bash
ros2 launch blueshift_web simple.launch.py
```

_Note: if there are any errors about missing packages, make download all dependencies as described [here](../README.md#install-dependencies)_

## Notes
Joystick functionality will only work correctly in a Chrome based browser (ie. Chrome or Chromium).

# Setup Instructions
### Install [nvm](https://github.com/nvm-sh/nvm)
`curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/master/install.sh | bash`

Open a new terminal to get access to `nvm`

### Install `node`
`nvm install 17`

We use, [`pnpm`](https://github.com/pnpm/pnpm) instead of `npm`

Install it with `npm install -g pnpm`

Then replace npm in all future commands with `pnpm` (this should work for any commands you find online too)

### Download all node packages
`pnpm install` (`pnpm i` for short)

### Run the website for development
`pnpm run dev` or `pnpm run dev -- --host` if it needs to be accessible on other computers on your network

### Running the website with ROS
To build it: `colcon build --symlink-install --packages-select blueshift_interface`
To run it `ros2 launch blueshift_interface simple.launch` (this also starts the rosbridge_server to facilitate JS<->ROS communication)

_Note: if there are any errors about missing packages, make download all dependencies as described [here](../../README.md#install-dependencies)_
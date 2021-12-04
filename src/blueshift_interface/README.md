# Setup Instructions
### Install [`nvm`](https://github.com/nvm-sh/nvm)
```
curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/master/install.sh | bash
```

Open a new terminal to get access to `nvm`

### Install `node`
```
nvm install 17
```

We use, [`pnpm`](https://github.com/pnpm/pnpm) instead of `npm`

Install it with 
```
npm install -g pnpm
```

`pnpm` is a drop-in replacement for `npm` but it is faster, uses less storage, and is generally better.
Any `npm` command should work the exact same if you just replace it with `pnpm`.

### Download all node packages
_Note_: All commands after this point need to be run in the `blueshift_interface` directory
`pnpm install` (`pnpm i` for short)

### Run the website for development
```
pnpm run dev
```
If it needs to be accessible on other computers on your network
```
pnpm run dev -- --host
```

### Running the website with ROS
To build it:
```
colcon build --symlink-install --packages-select blueshift_interface
```
To run it (this also starts the rosbridge_server to facilitate JS<->ROS communication)
```
ros2 launch blueshift_interface simple.launch
```

_Note: if there are any errors about missing packages, make download all dependencies as described [here](../../README.md#install-dependencies)_

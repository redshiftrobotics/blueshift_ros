# Setup Instructions

## Install [`nvm`](https://github.com/nvm-sh/nvm)

```bash
curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/master/install.sh | bash
```

Open a new terminal to get access to `nvm`

## Install `node`

```bash
nvm install 17
```

## Install `pnpm`

We use, [`pnpm`](https://github.com/pnpm/pnpm) instead of [`npm`](https://www.npmjs.com/about). Read why [here](https://betterprogramming.pub/the-case-for-pnpm-over-npm-or-yarn-2b221607119)

_In general, `pnpm` is a drop-in replacement for `npm` but it is faster, uses less storage, and is generally better.
Any `npm` command should work the exact same if you just replace it with `pnpm`._

Install it with:

```bash
npm install -g pnpm
```

## Download all node packages

```bash
pnpm install
```

_Note_: All of the following commands including this one need to be run in the `src/blueshift_interface` directory

## Run the website for development

```bash
pnpm run dev
```

If it needs to be accessible on other computers on your network

```bash
pnpm run dev -- --host
```

## Running the website with ROS

To build it:

```bash
colcon build --packages-select blueshift_web
```

_Note 1_: This first time it is built, this might take a long time, because all the node packages need to be copied over to the install folder. Every subsequent time, it should only copy over the difference, so it should be faster. _TODO later: there may be a way to just re-download them into the install directory which might be quicker._

_Note 2_: Building this with the `--symlink-install` flag seems to raise cause an issue with symlinks, so this package (and probably the whole workspace) should be built _without_ that flag.

To run it (this also starts the rosbridge_server to facilitate JS<->ROS communication)

```bash
ros2 launch blueshift_web simple.launch
```

_Note: if there are any errors about missing packages, make download all dependencies as described [here](../../README.md#install-dependencies)_

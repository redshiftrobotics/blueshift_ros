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

### Run the website locally for development
`pnpm run dev` or `pnpm run dev -- --host` if it needs to be accessible on other computers on your network
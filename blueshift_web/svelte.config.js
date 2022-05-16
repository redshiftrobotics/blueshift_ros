import preprocess from 'svelte-preprocess';

import { optimizeImports, optimizeCss } from 'carbon-preprocess-svelte';
import tsconfigPaths from 'vite-tsconfig-paths'

import adapter from '@sveltejs/adapter-node';

/** @type {import('@sveltejs/kit').Config} */
const config = {
	// Consult https://github.com/sveltejs/svelte-preprocess
	// for more information about preprocessors
	preprocess: [preprocess(), optimizeImports()],
	kit: {
		adapter: adapter({
			// default options are shown
			out: 'build',
			precompress: true,
		}),
		vite: {
			plugins: [
				//   process.env.NODE_ENV === "production" && optimizeCss()
				tsconfigPaths() // Give vite the ability to resolve imports using TypeScript's path mapping.
			],
			build: {
				target: [ 'es2020' ] // required to use generated typescript interfaces for ros2 message definitions
			}
		}
	}
};

export default config;

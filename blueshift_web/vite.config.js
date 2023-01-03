import { sveltekit } from '@sveltejs/kit/vite';

/** @type {import('vite').UserConfig} */
const config = {
	plugins: [sveltekit()],
	test: {
		include: ['src/**/*.{test,spec}.{js,ts}']
	},
	// This is currently necessary to prevent out of memory errors: https://github.com/vitejs/vite/issues/2433
	build: { sourcemap: false, rollupOptions: { cache: false, }, },
};

export default config;

<script lang="ts">
	import { notificationManager } from '$lib/notification_manager';

	import { onMount } from 'svelte';
	import { dev } from '$app/env';

	onMount(() => {
		window.onerror = function (msg, source, lineNo, columnNo, error) {
			notificationManager.addNotification({
				title: `${error.name}: ${error.message}`,
				level: 'error',
				subtitle: source,
				caption: `${lineNo}:${columnNo}`,
				type: 'permanent'
			});
			console.log(error.message, error.name);
		};

		setTimeout(() => {
			throw new Error('test');
		}, 500);
	});
</script>

<slot />

<style lang="scss">
	// @import 'carbon-components-svelte/css/all.scss';
	@import 'carbon-components-svelte/css/all.css';
</style>

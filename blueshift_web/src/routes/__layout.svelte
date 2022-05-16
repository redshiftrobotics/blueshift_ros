<script lang="ts">
	import { notificationManager } from '$lib/ts/notification_manager';

	import OverlayToastNotifications from '$lib/components/OverlayToastNotifications.svelte';

	import { onMount } from 'svelte';
	import { dev } from '$app/env';

	onMount(() => {
		// This generates notifications any errors that were generated from broken code
		window.onerror = function (msg, source, lineNo, columnNo, error) {
			notificationManager.addNotification({
				title: `JS ${error.name}: ${error.message}`,
				level: 'error',
				subtitle: `${source}:${lineNo}:${columnNo}`,
				caption: new Date().toLocaleString(),
				type: 'permanent'
			});
			return true;
		};

		// testing only
		// setTimeout(() => {
		// 	throw new Error('test');
		// }, 500);
	});
</script>

<slot />

<OverlayToastNotifications />

<style lang="scss">
	// @use '@carbon/styles';
	@import 'carbon-components-svelte/css/all.css';
</style>

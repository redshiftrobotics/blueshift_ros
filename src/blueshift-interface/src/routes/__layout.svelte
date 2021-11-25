<script lang="ts">
	import { notificationManager } from '$lib/notification_manager';

	import OverlayToastNotifications from '../components/OverlayToastNotifications.svelte';

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
		};

		// testing only
		setTimeout(() => {
			throw new Error('test');
		}, 500);
	});
</script>

<slot />

<OverlayToastNotifications />

<style lang="scss">
	// @import 'carbon-components-svelte/css/all.scss';
	@import 'carbon-components-svelte/css/all.css';
</style>

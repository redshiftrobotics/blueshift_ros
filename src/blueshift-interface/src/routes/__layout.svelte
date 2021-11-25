<script lang="ts">
	import { notificationManager } from '$lib/notification_manager';

	import OverlayToastNotifications from '../components/OverlayToastNotifications.svelte';

	import { onMount } from 'svelte';
	import { dev } from '$app/env';

	onMount(() => {
		// This handles any errors that were generated from broken code
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
			//console.warn('test');
			//console.trace();
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

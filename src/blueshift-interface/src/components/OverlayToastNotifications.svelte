<script lang="ts">
	import { InlineNotification } from 'carbon-components-svelte';

	import type { Notification } from '$lib/notification_manager';

	export let notifications: Notification[];
	export let displayTime = 5000;
</script>

<div class="box">
	<div
		style="
			position: absolute;
			right: 20px;
			bottom: 20px;
			width: 20rem;"
	>
		{#each notifications as notification}
			<InlineNotification
				lowContrast
				hideCloseButton
				title={notification.title}
				kind={notification.level}
				subtitle={notification.subtitle}
				timeout={displayTime}
				on:close={() => {
					if (notification.type === 'toast') {
						notifications = notifications.filter((n) => n.id !== notification.id);
					}
				}}
			/>
		{/each}
	</div>
</div>

<style>
	.box {
		position: relative;
		width: 100%;
		height: 100%;
	}
</style>

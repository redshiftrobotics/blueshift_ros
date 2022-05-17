<script lang="ts">
	import { InlineNotification } from 'carbon-components-svelte';

	import { notificationManager } from '$lib/ts/notification_manager';

	import { fly } from 'svelte/transition';
</script>

<div class="notification-container">
	<div
		style="
			position: fixed;
			right: 20px;
			bottom: 20px;
			width: 20rem;
			overflow: hidden;"
	>
		{#each $notificationManager as notification (notification.id)}
			{#if notification.type !== 'permanent-no-toast'}
				<div transition:fly={{ x: 300 }}>
					<InlineNotification
						hideCloseButton
						title={notification.title}
						kind={notification.level}
						subtitle={notification.subtitle}
					/>
				</div>
			{/if}
		{/each}
	</div>
</div>

<style>
	.notification-container {
		position: relative;
		width: 100%;
		height: 100%;
	}
</style>

<script lang="ts">
	import { InlineNotification } from 'carbon-components-svelte';

	import { notificationManager } from '$lib/notification_manager';
	import type { Notification } from '$lib/notification_manager';
import { loop_guard } from 'svelte/internal';
</script>

<div class="box">
	<div
		style="
			position: absolute;
			right: 20px;
			bottom: 20px;
			width: 20rem;"
	>
		{#each $notificationManager as notification}
			{#if notification.type !== 'permanent-no-toast'}
				<InlineNotification
					lowContrast
					hideCloseButton
					title={notification.title}
					kind={notification.level}
					subtitle={notification.subtitle}
				/>
			{/if}
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

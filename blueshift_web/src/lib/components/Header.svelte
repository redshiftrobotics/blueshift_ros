<script lang="ts">
	import { onMount } from 'svelte';
	import {
		Header,
		HeaderUtilities,
		HeaderActionLink,
		SkipToContent,
		HeaderGlobalAction,
		HeaderNav,
		ToastNotification,
		Theme
	} from 'carbon-components-svelte';
	import type { CarbonTheme } from 'carbon-components-svelte/types/Theme/Theme.svelte';

	import Settings20 from 'carbon-icons-svelte/lib/Settings20';
	import Debug20 from 'carbon-icons-svelte/lib/Debug20';
	import SettingsAdjust20 from 'carbon-icons-svelte/lib/SettingsAdjust20';
	import Drone20 from 'carbon-icons-svelte/lib/Drone20';
	import Power20 from 'carbon-icons-svelte/lib/Power20';
	import Restart20 from 'carbon-icons-svelte/lib/Restart20';

	import Maximize20 from 'carbon-icons-svelte/lib/Maximize20';
	import Minimize20 from 'carbon-icons-svelte/lib/Minimize20';

	import Sun20 from 'carbon-icons-svelte/lib/Sun20';
	import Moon20 from 'carbon-icons-svelte/lib/Moon20';
	import Question from 'carbon-pictograms-svelte/lib/Question.svelte';
	import Envelope from 'carbon-pictograms-svelte/lib/Envelope.svelte';
	import SelectProduct from 'carbon-pictograms-svelte/lib/SelectProduct.svelte';
	import CodeSyntax from 'carbon-pictograms-svelte/lib/CodeSyntax.svelte';

	import ErrorWarningStatus from './ErrorWarningStatus.svelte';
	import CustomHeaderAction from './CustomHeaderAction.svelte';
	import { notificationManager } from '$lib/ts/notification_manager';

	// Full screen handler (modified from: https://github.com/codechips/svelte-fullscreen-example)
	let fs = false;
	$: fullscreenIcon = fs ? Minimize20 : Maximize20;
	let fsToggle;

	onMount(() => {
		// boring plain js fullscreen support stuff below
		const noop = () => {};

		const fullscreenSupport = !!(
			document.fullscreenEnabled ||
			// @ts-ignore
			document.webkitFullscreenEnabled ||
			// @ts-ignore
			document.mozFullScreenEnabled ||
			// @ts-ignore
			document.msFullscreenEnabled ||
			false
		);

		const exitFullscreen = (
			document.exitFullscreen ||
			// @ts-ignore
			document.mozCancelFullScreen ||
			// @ts-ignore
			document.webkitExitFullscreen ||
			// @ts-ignore
			document.msExitFullscreen ||
			noop
		).bind(document);

		const requestFullscreen = (fsContainer) => {
			const requestFS = (
				fsContainer.requestFullscreen ||
				fsContainer.mozRequestFullScreen ||
				fsContainer.webkitRequestFullscreen ||
				fsContainer.msRequestFullscreen ||
				noop
			).bind(document.documentElement);
			requestFS();
		};

		fsToggle = () => {
			if (!fullscreenSupport) return;

			if (fs) {
				exitFullscreen();
			} else {
				requestFullscreen(document.documentElement);
			}
			fs = !fs;
		};
	});

	// Theme code
	const darkTheme: CarbonTheme = 'g100';
	const lightTheme: CarbonTheme = 'white';
	let theme: CarbonTheme = darkTheme;
	$: themeIcon = theme === lightTheme ? Moon20 : Sun20;

	// Notification panel state
	let isOpen = false;
</script>

<Theme bind:theme persist persistKey="__carbon-theme" />

<!-- TODO: Add logo here once its done -->
<Header company="[logo]" platformName="Blueshift Robotics">
	<div slot="skip-to-content">
		<SkipToContent />
	</div>
	<!-- Center buttons -->
	<!-- I don't know if this is the correct way to add extra header elements, but it works lol -->
	<HeaderNav>
		<!-- Add spacing to the right side with `--cds-spacing-05` otherwise they touch the divider -->
		<div style="padding-right: var(--cds-spacing-05)">
			<HeaderGlobalAction icon={Power20} />
			<HeaderGlobalAction icon={Restart20} />
		</div>

		{#if $$slots.middle_section}
			<!-- Add spacing to the right side with `--cds-spacing-05` otherwise they touch the divider -->
			<div style="padding-right: var(--cds-spacing-05)">
				<slot name="middle_section" />
			</div>
		{/if}
	</HeaderNav>

	<!-- Fullscreen Button -->
	<HeaderNav>
		<HeaderGlobalAction
			icon={fullscreenIcon}
			on:click={() => {
				fsToggle();
			}}
		/>
		<HeaderGlobalAction
			icon={themeIcon}
			on:click={() => {
				if (theme === lightTheme) {
					theme = darkTheme;
				} else {
					theme = lightTheme;
				}
			}}
		/>
	</HeaderNav>

	<!-- Right side buttons -->
	<HeaderUtilities>
		<HeaderActionLink icon={Drone20} href="/" />
		<HeaderActionLink icon={SettingsAdjust20} href="/copilot" />
		<HeaderActionLink icon={Debug20} href="/debug" />
		<HeaderActionLink icon={Settings20} href="/settings" />
		<HeaderNav>
			<!--
				CustomHeaderAction is based on the source HeaderAction with a few chances
				- `text` does not display any text, it just adds or removes the action-text class
				- all properties are passed down to the icons (this enables them to access `numErrors` and `numWarnings`)
			 -->
			<CustomHeaderAction
				numErrors={$notificationManager.filter((n) => n.level === 'error').length}
				numWarnings={$notificationManager.filter((n) =>
					['warning', 'warning-alt'].includes(n.level)
				).length}
				text="true"
				icon={ErrorWarningStatus}
				closeIcon={ErrorWarningStatus}
				bind:isOpen
			>
				{#if $notificationManager.length > 0}
					{#each $notificationManager as notification (notification.id)}
						{#if notification.type !== 'toast'}
							<ToastNotification
								lowContrast
								title={notification.title}
								kind={notification.level}
								subtitle={notification.subtitle}
								caption={notification.caption}
								style="width: 16rem; margin-top: 0;"
								on:close={() => {
									notificationManager.removeNotification(notification.id);
									// make sure the notification panel stays open when a notification is dismissed
									// TODO: this is a hack, find a better way to do this
									setTimeout(() => {
										isOpen = true;
									}, 1);
								}}
							/>
							<!-- width: 16; makes the notification the width of the panel -->
							<!-- margin-top: 0; removes the margin on the topmost notification so the spacing is even -->
						{/if}
					{/each}
				{:else}
					<div style="text-align: center; margin-top: var(--cds-spacing-09)">
						<!--
						<Question />
						<SelectProduct />
						<Envelope /> 
					-->
						<CodeSyntax />
						<p style="margin-top: var(--cds-spacing-04)">No notifications</p>
					</div>
				{/if}
			</CustomHeaderAction>
		</HeaderNav>
	</HeaderUtilities>
</Header>

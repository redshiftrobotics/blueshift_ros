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
		Theme,
	} from 'carbon-components-svelte';
	import type { CarbonTheme } from 'carbon-components-svelte/types/Theme/Theme.svelte';

	import Settings from 'carbon-icons-svelte/lib/Settings.svelte';
	import Debug from 'carbon-icons-svelte/lib/Debug.svelte';
	import SettingsAdjust from 'carbon-icons-svelte/lib/SettingsAdjust.svelte';
	import Drone from 'carbon-icons-svelte/lib/Drone.svelte';
	import Power from 'carbon-icons-svelte/lib/Power.svelte';
	import Restart from 'carbon-icons-svelte/lib/Restart.svelte';

	import Maximize from 'carbon-icons-svelte/lib/Maximize.svelte';
	import Minimize from 'carbon-icons-svelte/lib/Minimize.svelte';

	import Sun from 'carbon-icons-svelte/lib/Sun.svelte';
	import Moon from 'carbon-icons-svelte/lib/Moon.svelte';
	import Question from 'carbon-pictograms-svelte/lib/Question.svelte';
	import Envelope from 'carbon-pictograms-svelte/lib/Envelope.svelte';
	import SelectProduct from 'carbon-pictograms-svelte/lib/SelectProduct.svelte';
	import CodeSyntax from 'carbon-pictograms-svelte/lib/CodeSyntax.svelte';

	import ErrorWarningStatus from './ErrorWarningStatus.svelte';
	import CustomHeaderAction from './CustomHeaderAction.svelte';
	import { notificationManager } from '$lib/ts/notification_manager';

	// Full screen handler (modified from: https://github.com/codechips/svelte-fullscreen-example)
	let fs = false;
	$: fullscreenIcon = fs ? Minimize : Maximize;
	let fsToggle: () => void;

	onMount(() => {
		// boring plain js fullscreen support stuff below
		const noop = () => undefined;

		// TODO (kavidey): remove the @ts-ignore
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

		const requestFullscreen = (fsContainer: HTMLElement) => {
			const requestFS = (
				fsContainer.requestFullscreen ||
				// @ts-ignore
				fsContainer.mozRequestFullScreen ||
				// @ts-ignore
				fsContainer.webkitRequestFullscreen ||
				// @ts-ignore
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
	$: themeIcon = theme === lightTheme ? Moon : Sun;

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
			<HeaderGlobalAction icon={Power} />
			<HeaderGlobalAction icon={Restart} />
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
		<HeaderActionLink icon={Drone} href="/" />
		<HeaderActionLink icon={SettingsAdjust} href="/copilot" />
		<HeaderActionLink icon={Debug} href="/debug" />
		<HeaderActionLink icon={Settings} href="/settings" />
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
									if (!notification.id) {
										throw 'notification id is not defined';
									} else {
										notificationManager.removeNotification(notification.id);
									}

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

<script lang="ts">
	import {
		Content,
		Grid,
		Row,
		Column,
		HeaderNav,
		HeaderNavMenu,
		HeaderNavItem,
		HeaderGlobalAction
	} from 'carbon-components-svelte';

	import Grid20 from 'carbon-icons-svelte/lib/Grid20';
	import Checkbox20 from 'carbon-icons-svelte/lib/Checkbox20';

	import Header from '../components/Header.svelte';

	import { robotIP } from '$lib/ros_communication';
	import {
		GamepadState,
		registerGamepadConnectedListener,
		registerGamepadDisconnectedListener,
		setupGamepad
	} from '$lib/gamepad_communication';

	import type { Notification } from '$lib/notification_manager';
	import OverlayToastNotifications from '../components/OverlayToastNotifications.svelte';

	import type { Readable } from 'svelte/store';
	import { onMount } from 'svelte';

	let selectedCamera = 'Loading...';
	let cameras = [];
	let notifications: Notification[] = [
		{
			id: 0,
			title: 'Notification 1',
			subtitle: 'This is a notification',
			level: 'info',
			type: 'permanent'
		},
		{
			id: 1,
			title: 'Notification 2',
			subtitle: 'This is a notification',
			level: 'info',
			type: 'toast'
		}
	];

	let mode = 'one_cam';
	$: cameraIcon = mode == 'one_cam' ? Grid20 : Checkbox20;

	// Gamepad logic
	let gamepadState: Readable<GamepadState>;
	onMount(() => {
		registerGamepadConnectedListener((event: GamepadEvent) => {
			console.log('Gamepad connected', event.gamepad.id);

			gamepadState = setupGamepad(event.gamepad, 0.06);
		});

		registerGamepadDisconnectedListener((event: GamepadEvent) => {
			console.log('Gamepad disconnected', event.gamepad.id);
		});
	});
</script>

<Header bind:notifications>
	<HeaderNav slot="middle_section">
		{#if mode == 'one_cam'}
			<HeaderNavMenu text={selectedCamera}>
				{#each cameras as camera}
					<HeaderNavItem text={camera} href="#" />
				{/each}
			</HeaderNavMenu>
		{/if}
		<HeaderGlobalAction
			icon={cameraIcon}
			on:click={() => {
				if (mode == 'one_cam') {
					mode = 'multi_cam';
				} else {
					mode = 'one_cam';
				}
			}}
		/>
	</HeaderNav>
</Header>

{#if mode == 'one_cam'}
	<Content style="padding: var(--cds-spacing-05);">
		<!-- padding: var(--cds-spacing-05); decrease padding all around the camera display -->
		<Grid style="max-width: 100%">
			<Row>
				<Column aspectRatio="16x9" style="outline: 1px solid var(--cds-interactive-04)">16x9</Column
				>
			</Row>
		</Grid>
	</Content>
{:else if mode == 'multi_cam'}
	<Content style="padding: var(--cds-spacing-05);">
		<Grid style="max-width: 100%">
			<Row>
				<Column aspectRatio="16x9" style="outline: 1px solid var(--cds-interactive-04)">16x9</Column
				>
				<Column aspectRatio="16x9" style="outline: 1px solid var(--cds-interactive-04)">16x9</Column
				>
			</Row>
			<Row>
				<Column aspectRatio="16x9" style="outline: 1px solid var(--cds-interactive-04)">16x9</Column
				>
				<Column aspectRatio="16x9" style="outline: 1px solid var(--cds-interactive-04)">16x9</Column
				>
			</Row>
		</Grid>
	</Content>
{/if}

<OverlayToastNotifications bind:notifications />

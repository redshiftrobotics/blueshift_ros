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

	import Header from '$lib/components/Header.svelte';

	import {
		connectToROS,
		robotIP,
		websocketPort,
		getROSConnected,
		ROSLIB
	} from '$lib/ts/ros_communication';
	import {
		GamepadState,
		registerGamepadConnectedListener,
		registerGamepadDisconnectedListener,
		setupGamepad,
		getGamepadConnected
	} from '$lib/ts/gamepad_communication';

	import { notificationManager } from '$lib/ts/notification_manager';
	import type { Notification } from '$lib/ts/notification_manager';

	import type { Readable } from 'svelte/store';
	import { onMount } from 'svelte';

	import log from '$lib/ts/logger';

	// testing only
	// setTimeout(() => {
	// 	log.warn('test warning');
	// }, 1000);
	// setTimeout(() => {
	// 	log.error('test error');
	// }, 1500);

	let selectedCamera = 'Loading...';
	let cameras = [];

	// testing only
	// setTimeout(() => {
	// 	selectedCamera = 'Camera 1';
	// 	cameras = ['Camera 1', 'Camera 2', 'Camera 3', 'Camera 4', 'Camera 5'];
	// }, 1000);

	let mode = 'one_cam';
	$: cameraIcon = mode == 'one_cam' ? Grid20 : Checkbox20;

	let gamepadState: Readable<GamepadState>;
	let robotMovementTopic;
	onMount(async () => {
		registerGamepadConnectedListener((event: GamepadEvent) => {
			gamepadState = setupGamepad(event.gamepad, 0.06);

			notificationManager.addNotification({
				title: 'JS: Gamepad Connected',
				subtitle: event.gamepad.id,
				level: 'info',
				type: 'toast'
			});
		});

		registerGamepadDisconnectedListener((event: GamepadEvent) => {
			notificationManager.addNotification({
				title: 'JS: Gamepad Disconnected',
				subtitle: event.gamepad.id,
				level: 'info',
				type: 'toast'
			});
		});

		// Svelte and/or SvelteKit don't support top level await (https://github.com/sveltejs/svelte/issues/5501, https://github.com/sveltejs/kit/issues/941)
		// Because loading ROSLIB is async, everything that uses it has to also be async
		let rosWS = await connectToROS(
			() => {
				notificationManager.addNotification({
					title: 'ROS: Connected to websocket server',
					subtitle: `ws://${robotIP}:${websocketPort}`,
					level: 'success',
					type: 'toast'
				});
			},
			(error) => {
				notificationManager.addNotification({
					title: 'ROS: error connecting to websocket server',
					level: 'error',
					subtitle: error.target.url,
					caption: new Date().toLocaleString(),
					type: 'permanent'
				});
				log.debug(error);
			},
			() => {
				notificationManager.addNotification({
					title: 'ROS: Connection to websocket server closed',
					subtitle: `ws://${robotIP}:${websocketPort}`,
					level: 'error',
					type: 'toast'
				});
			}
		);

		/**
		 * TODO: Find a better way to work with ROSLIB
		 *
		 * Maybe create a custom set of functions for talking to topics, services, params, and actions?
		 *   This will probably be necessary if we want an automated way to turn topics that we listen to into stores (we might even be able to write to them that way too)
		 *
		 * Ideally we would also want typescript definitions for everything (both Topic and Message functions, as well as a way to generate definitions for every message/server type we use)
		 *
		 * We could also create a custom promise that either throws an error or resolves to a working websocket connection
		 */
		robotMovementTopic = new ROSLIB.Topic({
			ros: rosWS,
			name: '/topic',
			messageType: 'geometry_msgs/Twist'
		});

		gamepadState.subscribe((gamepad: GamepadState) => {
			if (getROSConnected() && getGamepadConnected()) {
				let twist = new ROSLIB.Message({
					linear: {
						x: gamepad.left.stick.x,
						y: gamepad.left.stick.y,
						z: Number(gamepad.left.bumper.pressed) - Number(gamepad.right.bumper.pressed)
					},
					angular: {
						x: gamepad.right.stick.x,
						y: gamepad.right.stick.y,
						z: gamepad.left.trigger - gamepad.right.trigger
					}
				});
				robotMovementTopic.publish(twist);
			}
		});
	});
</script>

<Header>
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
				<Column aspectRatio="16x9" style="outline: 1px solid var(--cds-interactive-04)">
					16x9
				</Column>
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

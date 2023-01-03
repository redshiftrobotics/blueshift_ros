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

	import Thumbnail_2 from "carbon-icons-svelte/lib/Thumbnail_2.svelte";
	import Checkbox from 'carbon-icons-svelte/lib/Checkbox.svelte';

	import Header from '$lib/components/Header.svelte';

	import type { GeometryTwist } from '$lib/ts/utils/ros_types';
	import {
		connectToROS,
		robotIP,
		websocketPort,
		ROSConnected,
		topic,
		service
	} from '$lib/ts/ros_communication';
	import type { Topic } from '$lib/ts/ros_communication';

	import {
		GamepadState,
		registerGamepadConnectedListener,
		registerGamepadDisconnectedListener,
		setupGamepad,
		gamepadConnected
	} from '$lib/ts/gamepad_communication';

	import { notificationManager } from '$lib/ts/notification_manager';
	import type { Notification } from '$lib/ts/notification_manager';

	import type { Readable, Writable } from 'svelte/store';
	import { onMount } from 'svelte';

	import log from '$lib/ts/logger';
	import WebRtcVideo from '$lib/components/WebRTCVideo.svelte';

	// testing only
	// setTimeout(() => {
	// 	log.warn('test warning');
	// }, 1000);
	// setTimeout(() => {
	// 	log.error('test error');
	// }, 1500);

	let selectedCamera = 'Loading...';
	let cameras: string[] = [];

	// testing only
	// setTimeout(() => {
	// 	selectedCamera = 'Camera 1';
	// 	cameras = ['Camera 1', 'Camera 2', 'Camera 3', 'Camera 4', 'Camera 5'];
	// }, 1000);

	let mode: 'one_cam' | 'multi_cam' = 'one_cam';

	let gamepadState: Readable<GamepadState>;

	onMount(async () => {
		registerGamepadConnectedListener((event: GamepadEvent) => {
			gamepadState = setupGamepad(event.gamepad.index, 0.06, 30);

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
	});

	const rosWebSocket = connectToROS(
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
	let robotMovementTopic = topic<GeometryTwist, 'publish'>(
		'/input',
		'geometry_msgs/Twist',
		'publish',
		{
			linear: { x: 0, y: 0, z: 0 },
			angular: { x: 0, y: 0, z: 0 }
		}
	);

	$: {
		if ($gamepadConnected && $ROSConnected) {
			$robotMovementTopic = {
				linear: {
					x: $gamepadState.left.stick.x,
					y: $gamepadState.left.stick.y,
					z: Number($gamepadState.left.bumper.pressed) - Number($gamepadState.right.bumper.pressed)
				},
				angular: {
					x: $gamepadState.right.stick.x,
					y: $gamepadState.right.stick.y,
					z: $gamepadState.left.trigger - $gamepadState.right.trigger
				}
			};
		}
	}
</script>

<Header>
	<HeaderNav slot="middle_section">
		{#if mode == 'one_cam'}
			<HeaderNavMenu text={selectedCamera}>
				{#each cameras as camera}
					<HeaderNavItem text={camera} href="#" />
				{/each}
			</HeaderNavMenu>
		{:else}
			<HeaderNavItem
				text="You're Doing Great"
				on:click={() => {
					notificationManager.addNotification({
						title: 'Yay!!',
						subtitle: "You're doing a great job",
						level: 'success',
						type: 'toast'
					});
				}}
			/>
		{/if}
		<HeaderGlobalAction
			icon={mode == 'one_cam' ? Thumbnail_2 : Checkbox}
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
					{$ROSConnected && $gamepadConnected} <br />
					{JSON.stringify($gamepadState)}
					<WebRtcVideo />
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

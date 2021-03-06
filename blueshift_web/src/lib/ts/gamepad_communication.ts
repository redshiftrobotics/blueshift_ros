import { readable } from 'svelte/store';
import type { Readable } from 'svelte/store';

class Button {
	pressed = false;
	justPressed = false;
}

class DPad {
	up = false;
	down = false;
	left = false;
	right = false;
}

class Stick {
	x = 0;
	y = 0;
	button: Button = new Button();
}

class GamepadSide {
	stick: Stick = new Stick();
	bumper: Button = new Button();
	trigger = 0;
}

class GamepadButtons {
	a: Button = new Button();
	b: Button = new Button();
	x: Button = new Button();
	y: Button = new Button();
	back: Button = new Button();
	start: Button = new Button();
	home: Button = new Button();
}

export class GamepadState {
	left: GamepadSide = new GamepadSide();
	right: GamepadSide = new GamepadSide();
	dpad: DPad = new DPad();
	buttons: GamepadButtons = new GamepadButtons();
}

/**
 * Registers a callback when the gamepad is connected
 *
 * @remarks
 * This must be called within onMount() or `window` will be undefined
 *
 * @param callback function to call when a gamepad is connected
 */
export function registerGamepadConnectedListener(callback: (event: GamepadEvent) => void): void {
	if (typeof window === 'undefined') {
		throw new Error('window is undefined. This must be called within onMount()');
	}
	window.addEventListener('gamepadconnected', (event: GamepadEvent) => {
		callback(event);
	});
}

/**
 * Registers a callback when the gamepad is disconnected
 *
 * @remarks
 * This must be called within onMount() or `window` will be undefined
 *
 * @param callback function to call when a gamepad is disconnected
 */
export function registerGamepadDisconnectedListener(callback: (event: GamepadEvent) => void): void {
	if (typeof window === 'undefined') {
		throw new Error('window is undefined. This must be called within onMount()');
	}
	window.addEventListener('gamepaddisconnected', (event: GamepadEvent) => {
		callback(event);
	});
}


/**
 * This svelte readable store keeps track of whether a gamepad is connected or not. It is updated automatically
 */
export const gamepadConnected = readable<boolean>(false, function start(set) {
	if (typeof window !== 'undefined') {
		window.addEventListener('gamepadconnected', () => {
			set(true);
		});
		window.addEventListener('gamepaddisconnected', () => {
			set(false);
		});
	}

	return () => undefined;
});

// TODO: update this to work with multiple gamepads (keep track of them by their id, etc.)
let lastGamepadState: GamepadState = new GamepadState();

/**
 * returns 0 for any number within the threshold, and the number otherwise
 * @param value value to threshold
 * @param threshold threshold value
 * @returns thresholded value
 */
function threshold(value: number, threshold: number): number {
	if (Math.abs(value) < threshold) {
		return 0;
	}
	return value;
}

/**
 * Gets the current state of a gamepad in human-readable format
 * @param gamepad gamepad object to get the state from
 * @param th deadband threshold range for analog inputs (joysticks, triggers)
 * @returns gamepad state
 */
function getGamepadState(gamepad: Gamepad, th: number): GamepadState {
	const state = new GamepadState();
	if (gamepadConnected) {
		state.left.stick.x = threshold(gamepad.axes[0], th);
		state.left.stick.y = threshold(gamepad.axes[1], th);
		state.left.stick.button.pressed = gamepad.buttons[0].pressed;
		state.left.stick.button.justPressed =
			gamepad.buttons[0].pressed && !lastGamepadState.left.stick.button.pressed;
		state.left.bumper.pressed = gamepad.buttons[5].pressed;
		state.left.bumper.justPressed =
			gamepad.buttons[5].pressed && !lastGamepadState.left.bumper.pressed;
		state.left.trigger = threshold(gamepad.buttons[6].value, th);
		state.right.stick.x = threshold(gamepad.axes[2], th);
		state.right.stick.y = threshold(gamepad.axes[3], th);
		state.right.stick.button.pressed = gamepad.buttons[1].pressed;
		state.right.stick.button.justPressed =
			gamepad.buttons[1].pressed && !lastGamepadState.right.stick.button.pressed;
		state.right.bumper.pressed = gamepad.buttons[4].pressed;
		state.right.bumper.justPressed =
			gamepad.buttons[4].pressed && !lastGamepadState.right.bumper.pressed;
		state.right.trigger = threshold(gamepad.buttons[7].value, th);
		state.dpad.up = gamepad.buttons[12].pressed;
		state.dpad.down = gamepad.buttons[13].pressed;
		state.dpad.left = gamepad.buttons[14].pressed;
		state.dpad.right = gamepad.buttons[15].pressed;
		state.buttons.a.pressed = gamepad.buttons[2].pressed;
		state.buttons.a.justPressed = gamepad.buttons[2].pressed && !lastGamepadState.buttons.a.pressed;
		state.buttons.b.pressed = gamepad.buttons[3].pressed;
		state.buttons.b.justPressed = gamepad.buttons[3].pressed && !lastGamepadState.buttons.b.pressed;
		state.buttons.x.pressed = gamepad.buttons[0].pressed;
		state.buttons.x.justPressed = gamepad.buttons[0].pressed && !lastGamepadState.buttons.x.pressed;
		state.buttons.y.pressed = gamepad.buttons[1].pressed;
		state.buttons.y.justPressed = gamepad.buttons[1].pressed && !lastGamepadState.buttons.y.pressed;
		state.buttons.back.pressed = gamepad.buttons[8].pressed;
		state.buttons.back.justPressed =
			gamepad.buttons[8].pressed && !lastGamepadState.buttons.back.pressed;
		state.buttons.start.pressed = gamepad.buttons[9].pressed;
		state.buttons.start.justPressed =
			gamepad.buttons[9].pressed && !lastGamepadState.buttons.start.pressed;
		state.buttons.home.pressed = gamepad.buttons[10].pressed;
		state.buttons.home.justPressed =
			gamepad.buttons[10].pressed && !lastGamepadState.buttons.home.pressed;

		lastGamepadState = state;
	}

	return state;
}

/**
 * Generates a readable store of the gamepad state
 * @param gamepadIndex gamepad id to get data from (0-3)
 * @param threshold threshold value for analog inputs (joysticks, triggers)
 * @param rate rate at which to update the gamepad state
 * @returns a readable store of the gamepad state
 */
export function setupGamepad(
	gamepadIndex: number,
	threshold = 0.05,
	rate = 60
): Readable<GamepadState> {
	if (!navigator.getGamepads()[gamepadIndex]) {
		throw "Invalid gamepad ID"
	} else {
		return readable(new GamepadState(), function start(set) {
			const interval = setInterval(() => {
				const gamepadState = getGamepadState(navigator.getGamepads()[gamepadIndex]!, threshold);
				set(gamepadState);
			}, 1000 / rate);
	
			return () => {
				clearInterval(interval);
			};
		});
	}
}

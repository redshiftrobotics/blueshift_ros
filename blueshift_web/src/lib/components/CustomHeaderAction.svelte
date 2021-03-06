<script lang="ts">
	/**
	 * @typedef {{ delay?: number; duration?: number; easing?: (t: number) => number; }} HeaderActionSlideTransition
	 */
	/** Set to `true` to open the panel */
	export let isOpen = false;
	/**
	 * Specify the icon to render
	 * @type {typeof import("svelte").SvelteComponent}
	 */
	export let icon: any = AppSwitcher20;
	/**
	 * Specify the icon to render when the action panel is open
	 * @type {typeof import("svelte").SvelteComponent}
	 */
	export let closeIcon: any = Close20;
	/**
	 * Specify the text
	 * Alternatively, use the named slot "text" (e.g., <div slot="text">...</div>)
	 * @type {string}
	 */
	export let text: string;
	/** Obtain a reference to the button HTML element */
	export let ref: HTMLButtonElement = undefined;
	/**
	 * Customize the panel transition (i.e., `transition:slide`)
	 * Set to `false` to disable the transition
	 * @type {false | HeaderActionSlideTransition}
	 */
	export let transition = { duration: 200 };
	import { createEventDispatcher } from 'svelte';
	import { slide } from 'svelte/transition';
	import Close20 from 'carbon-icons-svelte/lib/Close20/Close20.svelte';
	import AppSwitcher20 from 'carbon-icons-svelte/lib/AppSwitcher20/AppSwitcher20.svelte';
	import { Icon } from 'carbon-components-svelte';
	const dispatch = createEventDispatcher();
	let refPanel: HTMLDivElement;
</script>

<svelte:window
	on:click={({ target }) => {
		if (isOpen && !ref.contains(target) && !refPanel.contains(target)) {
			isOpen = false;
			dispatch('close');
		}
	}}
/>

<div>
	<button
		bind:this={ref}
		type="button"
		class:bx--header__action={true}
		class:bx--header__action--active={isOpen}
		class:action-text={text}
		{...$$restProps}
		on:click
		on:click={() => {
			isOpen = !isOpen;
			dispatch(isOpen ? 'open' : 'close');
		}}
	>
		<Icon render={icon} {...$$restProps} style={isOpen ? 'display: none' : ''} />
		<Icon render={closeIcon} {...$$restProps} style={!isOpen ? 'display: none' : ''} />
	</button>
	{#if isOpen}
		<div
			bind:this={refPanel}
			class:bx--header-panel={true}
			class:bx--header-panel--expanded={true}
			transition:slide={{
				...transition,
				duration: transition === false ? 0 : transition.duration
			}}
		>
			<slot />
		</div>
	{/if}
</div>

<style>
	.action-text {
		font-size: 16px;
		line-height: 20px;
		text-decoration: none;
		color: #fff;
		width: 100%;
		padding: 0 1rem;
	}
</style>

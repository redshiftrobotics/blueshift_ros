<script lang="ts">
	import Misuse from 'carbon-icons-svelte/lib/Misuse.svelte';
	import MisuseOutline from 'carbon-icons-svelte/lib/MisuseOutline.svelte';
	import WarningAlt from 'carbon-icons-svelte/lib/WarningAlt.svelte';
	import WarningAltFilled from 'carbon-icons-svelte/lib/WarningAltFilled.svelte';

	export let style: string;
	export let numWarnings = 0;
	export let numErrors = 0;
</script>

<div {style}>
	{#if numErrors > 0}
		<Misuse size={20} style="fill: var(--cds-support-01)" />
	{:else}
		<MisuseOutline size={20} style="fill: var(--cds-support-01)" />
	{/if}
	<span class="span-text" style="margin-right: var(--cds-spacing-04);">{numErrors}</span>
	{#if numWarnings > 0}
		<WarningAltFilled size={20} style="fill: var(--cds-support-03)" />
	{:else}
		<WarningAlt size={20} style="fill: var(--cds-support-03)" />
	{/if}
	<span class="span-text" style="margin-right: var(--cds-spacing-04);">{numWarnings}</span>
</div>

<style>
	.span-text {
		vertical-align: top;
		margin-left: var(--cds-spacing-01);
	}
</style>

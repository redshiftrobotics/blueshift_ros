<script>
	import Misuse20 from 'carbon-icons-svelte/lib/Misuse20';
	import MisuseOutline20 from 'carbon-icons-svelte/lib/MisuseOutline20';
	import WarningAlt20 from 'carbon-icons-svelte/lib/WarningAlt20';
	import WarningAltFilled20 from 'carbon-icons-svelte/lib/WarningAltFilled20';

	export let style;
	export let numWarnings = 0;
	export let numErrors = 0;
</script>

<div {style}>
	{#if numErrors > 0}
		<Misuse20 style="fill: var(--cds-support-01)" />
	{:else}
		<MisuseOutline20 style="fill: var(--cds-support-01)" />
	{/if}
	<span class="span-text" style="margin-right: var(--cds-spacing-04);">{numErrors}</span>
	{#if numWarnings > 0}
		<WarningAltFilled20 style="fill: var(--cds-support-03)" />
	{:else}
		<WarningAlt20 style="fill: var(--cds-support-03)" />
	{/if}
	<span class="span-text" style="margin-right: var(--cds-spacing-04);">{numWarnings}</span>
</div>

<style>
	.span-text {
		vertical-align: top;
		margin-left: var(--cds-spacing-01);
	}
</style>

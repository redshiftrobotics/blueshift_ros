<script>
	import { Header } from 'carbon-components-svelte';

	import Misuse20 from 'carbon-icons-svelte/lib/Misuse20';
	import MisuseOutline20 from 'carbon-icons-svelte/lib/MisuseOutline20';
	import WarningAlt20 from 'carbon-icons-svelte/lib/WarningAlt20';
	import WarningAltFilled20 from 'carbon-icons-svelte/lib/WarningAltFilled20';

	export let style;
</script>

<div style={style}>
	<MisuseOutline20 style="fill: var(--cds-support-01)" />
	<span class="span-text" style="margin-right: var(--cds-spacing-04);">1</span>
	<WarningAlt20 style="fill: var(--cds-support-03)" />
	<span class="span-text" style="margin-right: var(--cds-spacing-04);">1</span>
</div>

<style>
	.span-text {
		vertical-align: top;
		margin-left: var(--cds-spacing-01);
	}
</style>

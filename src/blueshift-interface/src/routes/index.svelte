<script>
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
	import Header from '../components/header.svelte';

	let selectedCamera = 'Loading...';
	let cameras = [];
	let notifications = [1, 2, 3];

	let mode = 'one_cam';
	$: cameraIcon = mode == 'one_cam' ? Grid20 : Checkbox20;
</script>

<Header bind:notifications>
	<HeaderNav slot="middle_section">
		<HeaderNavMenu text={selectedCamera}>
			{#each cameras as camera}
				<HeaderNavItem text={camera} href="#" />
			{/each}
		</HeaderNavMenu>
		<HeaderGlobalAction icon={cameraIcon} on:click={() => {
			if (mode == 'one_cam') {
				mode = 'multi_cam';
			} else {
				mode = 'one_cam';
			}
		}} />
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

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
	import Switcher20 from 'carbon-icons-svelte/lib/Switcher20';
	import Header from '../components/header.svelte';

	export let selectedCamera = 'Loading...';
	export let cameras = [];
</script>

<Header>
	<HeaderNav slot="middle_section">
		<HeaderNavMenu bind:text={selectedCamera}>
			{#each cameras as camera}
				<HeaderNavItem text={camera} href="#" />
			{/each}
		</HeaderNavMenu>
		<HeaderGlobalAction icon={Switcher20} href="/" />
	</HeaderNav>
</Header>

<Content>
	<Grid>
		<Row>
			<Column>
				<h1>Pilot</h1>
			</Column>
		</Row>
	</Grid>
</Content>

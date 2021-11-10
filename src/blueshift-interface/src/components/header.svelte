<script>
	import {
		Header,
		HeaderUtilities,
		HeaderActionLink,
		SkipToContent,
		HeaderNavItem,
		HeaderNavMenu,
		HeaderAction,
		ButtonSet,
		Button,
		ContentSwitcher,
		Switch,
		TooltipIcon,
		HeaderGlobalAction,
		HeaderNav
	} from 'carbon-components-svelte';
	import Settings20 from 'carbon-icons-svelte/lib/Settings20';
	import Debug20 from 'carbon-icons-svelte/lib/Debug20';
	import SettingsAdjust20 from 'carbon-icons-svelte/lib/SettingsAdjust20';
	import Drone20 from 'carbon-icons-svelte/lib/Drone20';
	import Power20 from 'carbon-icons-svelte/lib/Power20';
	import Restart20 from 'carbon-icons-svelte/lib/Restart20';
	import Misuse20 from 'carbon-icons-svelte/lib/Misuse20';
	import MisuseOutline20 from 'carbon-icons-svelte/lib/MisuseOutline20';
	import WarningAlt20 from 'carbon-icons-svelte/lib/WarningAlt20';
	import WarningAltFilled20 from 'carbon-icons-svelte/lib/WarningAltFilled20';

	import ErrorWarningStatus from '../components/error_warning_status.svelte';
	import CustomHeaderAction from '../components/CustomHeaderAction.svelte';
</script>

<Header company="Blueshift" platformName="Robotics">
	<div slot="skip-to-content">
		<SkipToContent />
	</div>
	<!-- Center buttons -->
	<!-- I don't know if this is the correct way to add extra header elements, but it works lol -->
	<HeaderNav>
		<!-- Add spacing to the right side with `--cds-spacing-05` otherwise they touch the divider -->
		<div style="padding-right: var(--cds-spacing-05)">
			<HeaderGlobalAction icon={Power20} />
			<HeaderGlobalAction icon={Restart20} />
		</div>

		{#if $$slots.middle_section}
			<!-- Add spacing to the right side with `--cds-spacing-05` otherwise they touch the divider -->
			<div style="padding-right: var(--cds-spacing-05)">
				<slot name="middle_section" />
			</div>
		{/if}
	</HeaderNav>

	<!-- Right side buttons -->
	<HeaderUtilities>
		<HeaderActionLink icon={Drone20} href="/" />
		<HeaderActionLink icon={SettingsAdjust20} href="/copilot" />
		<HeaderActionLink icon={Debug20} href="/debug" />
		<HeaderActionLink icon={Settings20} href="/settings" />
		<HeaderNav>
			<!--
				this custom style is equivalent to action-text
				it needs to be applied to the button element inside of HeaderAction for the text to show up
			 -->
			<HeaderAction
				style="font-size: 16px;
					line-height: 20px;
					text-decoration: none;
					color: #fff;
					width: 100%;
					padding: 0 1rem;"
				icon={ErrorWarningStatus}
				closeIcon={ErrorWarningStatus}
			/>
			<!-- <HeaderAction icon={MisuseOutline20} text="10" closeIcon={MisuseOutline20} /> -->
			<!-- icon and closeIcon should switch to the filled in version and add error/warning counter when there are errors/warnings -->
			<!-- <HeaderAction icon={WarningAlt20} closeIcon={WarningAlt20} /> -->
		</HeaderNav>
	</HeaderUtilities>
</Header>

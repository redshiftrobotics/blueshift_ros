<script>
	import {
		Header,
		HeaderUtilities,
		HeaderActionLink,
		SkipToContent,
		HeaderGlobalAction,
		HeaderNav,
		ToastNotification
	} from 'carbon-components-svelte';
	import Settings20 from 'carbon-icons-svelte/lib/Settings20';
	import Debug20 from 'carbon-icons-svelte/lib/Debug20';
	import SettingsAdjust20 from 'carbon-icons-svelte/lib/SettingsAdjust20';
	import Drone20 from 'carbon-icons-svelte/lib/Drone20';
	import Power20 from 'carbon-icons-svelte/lib/Power20';
	import Restart20 from 'carbon-icons-svelte/lib/Restart20';

	import Question from 'carbon-pictograms-svelte/lib/Question.svelte';
	import Envelope from 'carbon-pictograms-svelte/lib/Envelope.svelte';
	import SelectProduct from 'carbon-pictograms-svelte/lib/SelectProduct.svelte';
	import CodeSyntax from 'carbon-pictograms-svelte/lib/CodeSyntax.svelte';

	import ErrorWarningStatus from '../components/error_warning_status.svelte';
	import CustomHeaderAction from '../components/CustomHeaderAction.svelte';

	export let notifications = [];
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
				CustomHeaderAction is based on the source HeaderAction with a few chances
				- `text` does not display any text, it just adds or removes the action-text class
				- all properties are passed down to the icons (this enables them to access `numErrors` and `numWarnings`)
			 -->
			<CustomHeaderAction
				numErrors="1"
				numWarnings="1"
				text="true"
				icon={ErrorWarningStatus}
				closeIcon={ErrorWarningStatus}
			>
				{#if notifications.length > 0}
						{#each notifications as notification}
							<ToastNotification
								title="Error"
								subtitle="An internal server error occurred."
								caption={notification}
								style="width: 16rem; margin-top: 0;" 
							/>
							<!-- width: 16; makes the notification the width of the panel -->
							<!-- margin-top: 0; removes the margin on the topmost notification so the spacing is even -->
						{/each}
				{:else}
					<div style="text-align: center; margin-top: var(--cds-spacing-09)">
						<!--
						<Question />
						<SelectProduct />
						<Envelope /> 
					-->
						<CodeSyntax />
						<p style="margin-top: var(--cds-spacing-04)">No notifications</p>
					</div>
				{/if}
			</CustomHeaderAction>
		</HeaderNav>
	</HeaderUtilities>
</Header>

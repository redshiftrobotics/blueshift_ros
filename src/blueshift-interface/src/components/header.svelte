<script>
	import {
		Header,
		HeaderUtilities,
		HeaderActionLink,
		SkipToContent
	} from 'carbon-components-svelte';
	import Settings20 from 'carbon-icons-svelte/lib/Settings20';
	import Debug20 from 'carbon-icons-svelte/lib/Debug20';
	import SettingsAdjust20 from 'carbon-icons-svelte/lib/SettingsAdjust20';
	import Drone20 from 'carbon-icons-svelte/lib/Drone20';
</script>

<Header company="Blueshift" platformName="Robotics">
	<div slot="skip-to-content">
		<SkipToContent />
	</div>
	<HeaderUtilities>
		<HeaderActionLink icon={Drone20} href="/" />
		<HeaderActionLink icon={SettingsAdjust20} href="/copilot" />
		<HeaderActionLink icon={Debug20} href="/debug" />
		<HeaderActionLink icon={Settings20} href="/settings" />
	</HeaderUtilities>
</Header>

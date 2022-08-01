<script lang="ts">
	import log from '$lib/ts/logger';
	import { onDestroy } from 'svelte';

	const pc = new RTCPeerConnection({
		sdpSemantics: 'unified-plan'
	});

	let video: HTMLVideoElement;

	async function negotiate() {
		pc.addTransceiver('video', { direction: 'recvonly' });
		pc.addTransceiver('audio', { direction: 'recvonly' });

		const offer = await pc.createOffer();
		await pc.setLocalDescription(offer);

		await new Promise<void>((resolve, reject) => {
			if (pc.iceGatheringState === 'complete') {
				resolve();
			} else {
				function checkState() {
					if (pc.iceGatheringState === 'complete') {
						pc.removeEventListener('icegatheringstatechange', checkState);
						resolve();
					}
				}
				pc.addEventListener('icegatheringstatechange', checkState);
			}
		});
		const response = await fetch('http://localhost:8080/offer', {
			body: JSON.stringify({
				sdp: pc.localDescription!.sdp,
				type: pc.localDescription!.type
			}),
			headers: {
				'Content-Type': 'application/json',
				'Access-Control-Allow-Origin': '*'
			},
			method: 'POST'
		});
        await pc.setRemoteDescription(await response.json());
	}

	function start() {
		pc.addEventListener('track', function (evt) {
			if (evt.track.kind == 'video') {
				video.srcObject = evt.streams[0];
			} else {
				log.warn(`Unexpected track of type ${evt.track.kind} added`);
			}
		});

		negotiate();
	}

	start();
	onDestroy(() => {
		pc.close();
	});
</script>

<!-- svelte-ignore a11y-media-has-caption -->
<video bind:this={video} autoplay={true} playsinline={true} />

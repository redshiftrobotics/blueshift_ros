<script lang="ts">
	import log from '$lib/ts/logger';
	import { onDestroy } from 'svelte';
	import { service } from '$lib/ts/ros_communication';

	const pc = new RTCPeerConnection({
		sdpSemantics: 'unified-plan',
		iceServers: [{urls: ['stun:stun.l.google.com:19302']}]
	});

	let video: HTMLVideoElement;

	async function negotiate() {
		pc.addTransceiver('video', { direction: 'recvonly' });
		pc.addTransceiver('audio', { direction: 'recvonly' });

		pc.oniceconnectionstatechange = () => console.log(pc.iceConnectionState);
		pc.onnegotiationneeded = () => console.log('negotiationneeded');
		pc.onicecandidate = ({candidate}) => console.log(candidate);
		pc.onsignalingstatechange = () => console.log(pc.signalingState);
		pc.onicecandidateerror = (icecandidateerror) => console.log(icecandidateerror);

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
		// const response = await fetch('http://localhost:8080/offer', {
		// 	body: JSON.stringify({
		// 		sdp: pc.localDescription!.sdp,
		// 		type: pc.localDescription!.type
		// 	}),
		// 	headers: {
		// 		'Content-Type': 'application/json',
		// 		'Access-Control-Allow-Origin': '*'
		// 	},
		// 	method: 'POST'
		// });

		// pc.addIceCandidate({
		// 	candidate: 'candidate:0 1 UDP 2122154243 10.211.55.5 53421 typ host',
		// 	sdpMid: '0'
		// }).catch((e) => {
		// 	console.log(`Failure during addIceCandidate(): ${e.name}`);
		// });
		const response = await service<
			{ sdp: string; type: string },
			{ sdp: string; type: RTCSdpType }
		>('web_RTC_offer_communication', 'blueshift_interfaces/WebRTCOfferCommunication', {
			sdp: pc.localDescription!.sdp,
			type: pc.localDescription!.type
		});
		console.log(response.sdp);
		await pc.setRemoteDescription(response);
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

	setTimeout(start, 1500);
	onDestroy(() => {
		pc.close();
	});
</script>

<!-- svelte-ignore a11y-media-has-caption -->
<video bind:this={video} autoplay={true} playsinline={true} />

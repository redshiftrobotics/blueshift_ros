import { browser } from '$app/environment';
import { readable, writable } from 'svelte/store';
import type { Readable, Writable } from 'svelte/store';

// Import ROSLIB using a custom shim that provides module syntax (importing it normally will throw an error)
import ROSLIB from "$lib/ts/utils/roslib.shim";

type publish = 'publish';
type subscribe = 'subscribe';
type publishSubscribe = publish | subscribe;
type ReadableWriteableStore<T, direction extends publishSubscribe> = direction extends subscribe
	? Readable<T>
	: Writable<T>;

// Only get the robot ip (which is equivalent to the website hostname) if this is running on the client (not if its being pre-rendered on the server)
export let robotIP: string;
if (browser) {
	robotIP = window.location.hostname;
}
export const websocketPort = '9090';

/**
 * This private function is used to set the state of the `ROSConnected` store
 * @param state _true_: connected, _false_: not connected
 */
let setROSConnected: (state: boolean) => void;

/**
 * This svelte readable store keeps track of whether we are connected to ROS or not. It is updated automatically
 */
export const ROSConnected: Readable<boolean> = readable<boolean>(false, function start(set) {
	setROSConnected = set;

	return () => undefined;
});

let ros: ROSLIB.Ros;

/**
 * Sets up communication with ROS (the robot's ip and default port of 9090 are used for the websocket communication)
 * @param onconnection callback on connection to ros websocket server
 * @param onerror callback if connection with ros websocket server fail
 * @param onclose callback on disconnection from ros websocket server
 * @returns ROSLIB.Ros object for communicating with ROS
 *
 * TODO (kavidey): figure out what type of error is passed to this callback
 */
export function connectToROS(
	onconnection?: () => void,
	onerror?: (error: any) => void,
	onclose?: () => void
): ROSLIB.Ros {
	// Setup the connection with ROS
	ros = new ROSLIB.Ros({
		url: `ws://${robotIP}:${websocketPort}`
	});

	// Register Event handlers
	ros.on('connection', function () {
		setROSConnected(true);

		if (onconnection) onconnection();
	});

	ros.on('error', function (error) {
		setROSConnected(false);

		if (onerror) onerror(error);
	});

	ros.on('close', function () {
		setROSConnected(false);

		if (onclose) onclose();
	});

	return ros;
}

/**
 * Creates a svelte store that can be used to interact with ROS topics
 * @param topicName the name of the topic to publish/subscribe to
 * @param messageType the name of the message type of the topic (this is the ROS message name, not typscript, ex: std_msgs/String or geometry_msgs/Twist)
 * @param communicationDirection 'publish' or 'subscribe'
 * @param initialValue the initial value of the store
 * @returns a svelte readable/writable store that can be used to send or recieve messages on a ROS topic
 * 
 * @remarks
 * If `communicationDirection` is 'publish', the returned store is a writable store. If `communicationDirection` is 'subscribe', the returned store is a readable store.
 */
export function topic<T, direction extends publishSubscribe>(
	topicName: string,
	messageType: string,
	communicationDirection: direction,
	initialValue: T
): ReadableWriteableStore<T, direction> {
	const rosTopic = new ROSLIB.Topic<T>({
		ros: ros,
		name: topicName,
		messageType: messageType
	});

	if (communicationDirection == 'publish') {
		const writeableTopicStore = writable<T>(initialValue);
		writeableTopicStore.subscribe((msg) => {
			rosTopic.publish(msg);
		});
		return writeableTopicStore;
	} else {
		const readableTopicStore = readable<T>(initialValue, function start(set) {
			rosTopic.subscribe((msg) => {
				set(msg);
			});
		});

		// for some reason the compiler thinks that the type here will be wrong, so we have to set it manually with as
		return readableTopicStore as ReadableWriteableStore<T, direction>;
	}
}

export type Topic<T, direction extends publishSubscribe> = ReadableWriteableStore<T, direction>;

/**
 * Requsest and recieve data from a ROS service asynchronously
 * @param serviceName the name of the topic to publish/subscribe to (ex: '/add_two_ints')
 * @param messageType the name of the message type of the service (this is the ROS message name, not typscript, ex: std_msgs/String or geometry_msgs/Twist)
 * @param request the request to send to the service, of type `req`
 * @returns the response from the service, of type `res`
 * 
 * @example await service<{a: number, b: number}, {sum: number}>('/add_two_ints', 'example_interfaces/AddTwoInts', {a: 3, b: 4})
 */
export async function service<req, res>(
	serviceName: string,
	serviceType: string,
	request: req
): Promise<res> {
	return new Promise<res>((resolve, reject) => {
		var client = new ROSLIB.Service({
			ros: ros,
			name: serviceName,
			serviceType: serviceType
		});
	
		var serviceRequest = new ROSLIB.ServiceRequest(request);
	
		client.callService(serviceRequest, result => {
			resolve(result)
		});
	})
}
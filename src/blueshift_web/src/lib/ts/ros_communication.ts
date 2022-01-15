import { browser } from '$app/env';
import { readable, writable } from 'svelte/store';
import type { Readable, Writable } from 'svelte/store';

import 'roslib'; // this creates a global ROSLIB varible that is accessed via `window.ROSLIB`

// Import the compiled typescript interface definitions from blushift_interfaces
import type {
        ROSMessageStrings,
        ROSMessage,
        ROSMessagesTypeTSDefinitions
} from 'blueshift_interfaces/interface_definitions';
import { ROSMessageFactories } from 'blueshift_interfaces/interface_definitions';

type publish = "publish";
type subscribe = "subscribe";
type publishSubscribe = publish | subscribe;
type ReadableWriteableStore<T, direction extends publishSubscribe> =
    direction extends subscribe ? Readable<T> : Writable<T>;

// Only get the robot ip (which is equivalent to the website hostname) if this is running on the client (not if its being pre-rendered on the server)
export let robotIP: string;
if (browser) {
    robotIP = window.location.hostname;
}
export const websocketPort = "9090";

/**
 * This private function is used to set the state of the `ROSConnected` store
 * @param state _true_: connected, _false_: not connected
 */
let setROSConnected = (state: Boolean) => null;

/**
 * This svelte readable store keeps track of whether we are connected to ROS or not. It is updated automatically
 */
export const ROSConnected: Readable<Boolean> = readable(false, function start(set) {
    setROSConnected = set;

    return function stop() { };
});

let ros;

/**
 * Sets up communication with ROS (the robot's ip and default port of 9090 are used for the websocket communication)
 * @param onconnection callback on connection to ros websocket server
 * @param onerror callback if connection with ros websocket server faile
 * @param onclose callback on disconnection from ros websocket server
 * @returns ROSLIB.Ros object for communicating with ROS
 */
export function connectToROS(onconnection = (): void => { }, onerror = (error): void => { }, onclose = (): void => { }) {
    // Setup the connection with ROS
    ros = new window.ROSLIB.Ros({
        url: `ws://${robotIP}:${websocketPort}`
    });

    // Register Event handlers
    ros.on('connection', function () {
        setROSConnected(true)
        onconnection();
    });

    ros.on('error', function (error) {
        setROSConnected(false)
        onerror(error);
    });

    ros.on('close', function () {
        setROSConnected(false)
        onclose();
    });

    return ros
}

export function topic<T extends ROSMessageStrings, direction extends publishSubscribe>(topicName: string, type: T, communicationDirection: direction): ReadableWriteableStore<ROSMessage<T>, direction> {
    const rosTopic = new window.ROSLIB.Topic({
        ros: ros,
        name: topicName,
        messageType: type
    });

    if (communicationDirection == "publish") {
        const writeableTopicStore = writable(ROSMessageFactories[type]() as ROSMessage<T>);
        writeableTopicStore.subscribe((msg: ROSMessage<T>) => {
            rosTopic.publish(new window.ROSLIB.Message(msg));
        });
        return writeableTopicStore;
    } else {
        const readableTopicStore = readable(ROSMessageFactories[type]() as ROSMessage<T>, function start(set) {
            rosTopic.subscribe((msg) => {
                set(msg as ROSMessage<T>);
            });
        });
        // for some reason the compiler thinks that the type here will be wrong, so we have to set it manually with as
        return readableTopicStore as ReadableWriteableStore<ROSMessage<T>, direction>;
    }
}

export type Topic<T extends ROSMessageStrings, direction extends publishSubscribe> = ReadableWriteableStore<ROSMessagesTypeTSDefinitions[T], direction>;

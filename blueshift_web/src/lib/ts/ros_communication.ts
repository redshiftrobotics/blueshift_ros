import { browser } from '$app/env';
import { readable, writable } from 'svelte/store';
import type { Readable, Writable } from 'svelte/store';

import * as ROSLIB from "roslib";

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
export function connectToROS(onconnection?: () => void, onerror?: (error: any) => void, onclose ?: () => void): ROSLIB.Ros {
    // Setup the connection with ROS
    ros = new ROSLIB.Ros({
        url: `ws://${robotIP}:${websocketPort}`
    });

    // Register Event handlers
    ros.on('connection', function () {
        setROSConnected(true);

        if (onconnection)
            onconnection();
    });

    ros.on('error', function (error) {
        setROSConnected(false)

        if (onerror)
            onerror(error);
    });

    ros.on('close', function () {
        setROSConnected(false)

        if (onclose)
            onclose();
    });

    return ros
}

export function topic<T, direction extends publishSubscribe>(topicName: string, messageType: string, communicationDirection: direction): ReadableWriteableStore<T, direction> {
    const rosTopic = new ROSLIB.Topic<T>({
        ros: ros,
        name: topicName,
        messageType: messageType
    });

    if (communicationDirection == "publish") {
        const writeableTopicStore = writable<T>();
        writeableTopicStore.subscribe((msg) => {
            rosTopic.publish(msg);
        });
        return writeableTopicStore;
    } else {
        const readableTopicStore = readable<T>(undefined, function start(set) {
            rosTopic.subscribe((msg) => {
                set(msg);
            });
        });
        
        // for some reason the compiler thinks that the type here will be wrong, so we have to set it manually with as
        return readableTopicStore  as ReadableWriteableStore<T, direction> ;
    }
}

export type Topic<T, direction extends publishSubscribe> = ReadableWriteableStore<T, direction>;

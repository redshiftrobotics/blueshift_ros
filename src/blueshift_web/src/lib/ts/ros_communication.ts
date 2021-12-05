import { browser } from '$app/env';
import { readable, writable } from 'svelte/store';
import type { Readable, Writable } from 'svelte/store';

// Only get the robot ip (which is equivalent to the website hostname) if this is running on the client (not if its being pre-rendered on the server)
export let robotIP: string;
if (browser) {
    robotIP = window.location.hostname;
}
export const websocketPort = "9090";

// This tells typescript that the window object can have a property called ROSLIB
declare global {
    interface Window { ROSLIB: any; }
}

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


export let ROSLIB: any;

let ros;

/**
 * Sets up communication with ROS (the robot's ip and default port of 9090 are used for the websocket communication)
 * @param onconnection callback on connection to ros websocket server
 * @param onerror callback if connection with ros websocket server faile
 * @param onclose callback on disconnection from ros websocket server
 * @returns ROSLIB.Ros object for communicating with ROS
 */
export async function connectToROS(onconnection = (): void => { }, onerror = (error): void => { }, onclose = (): void => { }) {
    if (browser) {
        // roslib creates a global variable called `ROSLIB`
        await import('roslib/build/roslib');

        // typescript doesn't know this variable exists, tricks it into thinking it does
        window.ROSLIB = window.ROSLIB || {};

        // This exports the ROSLIB variable so it can be used in other files
        ROSLIB = window.ROSLIB;

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
}

interface ROSMessageBase {
}

export interface geometry_msgs_Linear extends ROSMessageBase {
    x: number;
    y: number;
    z: number;
}

function geometry_msgs_Linear_Factory(): geometry_msgs_Linear {
    return {
        x: 0,
        y: 0,
        z: 0
    };
}

export interface geometry_msgs_Angular extends ROSMessageBase {
    x: number;
    y: number;
    z: number;
}

function geometry_msgs_Angular_Factory(): geometry_msgs_Angular {
    return {
        x: 0,
        y: 0,
        z: 0
    };
}

export interface geometry_msgs_Twist extends ROSMessageBase {
    linear: geometry_msgs_Linear;
    angular: geometry_msgs_Angular;
}

function geometry_msgs_Twist_Factory(): geometry_msgs_Twist {
    return {
        linear: geometry_msgs_Linear_Factory(),
        angular: geometry_msgs_Angular_Factory()
    };
}

type messageTypes = "geometry_msgs/Linear" | "geometry_msgs/Angular" | "geometry_msgs/Twist";

type ROSMessagesTypes = {
    "geometry_msgs/Linear": geometry_msgs_Linear,
    "geometry_msgs/Angular": geometry_msgs_Angular,
    "geometry_msgs/Twist": geometry_msgs_Twist
}

// https://fettblog.eu/typescript-type-maps/
// https://blog.rsuter.com/how-to-instantiate-a-generic-type-in-typescript/
type ROSMessage<T extends messageTypes> =
    T extends keyof ROSMessagesTypes ? ROSMessagesTypes[T] :
    ROSMessageBase;

let AllROSMessages = {
    "geometry_msgs/Linear": geometry_msgs_Linear_Factory,
    "geometry_msgs/Angular": geometry_msgs_Angular_Factory,
    "geometry_msgs/Twist": geometry_msgs_Twist_Factory
}

export function topic<T extends messageTypes>(topicName: string, type: T, communicationDirection: "publish" | "subscribe"): Writable<ROSMessage<T>> {
    const rosTopic = new window.ROSLIB.Topic({
        ros: ros,
        name: topicName,
        messageType: type
    });

    const sampleMessage: ROSMessage<T> = AllROSMessages[type]();

    const topicStore = writable(sampleMessage);

    if (communicationDirection == "publish") {
        topicStore.subscribe((msg: ROSMessage<T>) => {
            rosTopic.publish(new window.ROSLIB.Message(msg));
        });
    } else {
        rosTopic.subscribe((msg) => {
            topicStore.set(msg);
        });
    }

    return topicStore;
}
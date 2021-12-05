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

class ROSMessageBase {
}

export class geometry_msgs_Linear extends ROSMessageBase {
    x: number;
    y: number;
    z: number;

    constructor(x = 0, y = 0, z = 0) {
        super();
        this.x = x;
        this.y = y;
        this.z = z;
    }
}

export class geometry_msgs_Angular extends ROSMessageBase {
    x: number;
    y: number;
    z: number;

    constructor(x = 0, y = 0, z = 0) {
        super();
        this.x = x;
        this.y = y;
        this.z = z;
    }
}

export class geometry_msgs_Twist extends ROSMessageBase {
    linear: geometry_msgs_Linear;
    angular: geometry_msgs_Angular;

    constructor(linear = new geometry_msgs_Linear(), angular = new geometry_msgs_Angular()) {
        super();
        this.linear = linear;
        this.angular = angular;
    }
}

type messageTypes = "geometry_msgs/Linear" | "geometry_msgs/Angular" | "geometry_msgs/Twist";

type AllROSMessagesTypes = {
    "geometry_msgs/Linear": geometry_msgs_Linear,
    "geometry_msgs/Angular": geometry_msgs_Angular,
    "geometry_msgs/Twist": geometry_msgs_Twist
}

type ROSMessage<T extends messageTypes> =
    T extends keyof AllROSMessagesTypes ? AllROSMessagesTypes[T] :
    ROSMessageBase;

// class Factory {
//     create<T>(type: (new () => T)): T {
//         return new type();
//     }
// }
// let factory = new Factory();

let AllROSMessages: {[name: string] : ROSMessageBase} = {
    "geometry_msgs/Linear": new geometry_msgs_Linear(),
    "geometry_msgs/Angular": new geometry_msgs_Angular(),
    "geometry_msgs/Twist": new geometry_msgs_Twist()
}

// Copied from: https://gist.github.com/sunnyy02/2477458d4d1c08bde8cc06cd8f56702e#file-deepclone-ts
class cloneable {
  public static deepCopy<T>(source: T): T {
    return Array.isArray(source)
    ? source.map(item => this.deepCopy(item))
    : source instanceof Date
    ? new Date(source.getTime())
    : source && typeof source === 'object'
          ? Object.getOwnPropertyNames(source).reduce((o, prop) => {
             Object.defineProperty(o, prop, Object.getOwnPropertyDescriptor(source, prop)!);
             o[prop] = this.deepCopy((source as { [key: string]: any })[prop]);
             return o;
          }, Object.create(Object.getPrototypeOf(source)))
    : source as T;
  }
}

export function topic<T extends messageTypes>(topicName: string, type: T, communicationDirection: "publish" | "subscribe"): Writable<ROSMessage<T>> {
    const rosTopic = new window.ROSLIB.Topic({
        ros: ros,
        name: topicName,
        messageType: type
    });

    const sampleMessage = cloneable.deepCopy(AllROSMessages[type]);

    const topicStore = writable(sampleMessage);

    if (communicationDirection == "publish") {
        topicStore.subscribe((msg: typeof sampleMessage) => {
            rosTopic.publish(new window.ROSLIB.Message(msg));
        });
    } else {
        rosTopic.subscribe((msg) => {
            topicStore.set(msg);
        });
    }

    return topicStore;
}

/*

        robotMovementTopic = new ROSLIB.Topic({
            ros: rosWS,
            name: '/topic',
            messageType: 'geometry_msgs/Twist'
        });
    });

    $: {
        if ($gamepadConnected && $ROSConnected) {
            let twist = new ROSLIB.Message({
                linear: {
                    x: $gamepadState.left.stick.x,
                    y: $gamepadState.left.stick.y,
                    z: Number($gamepadState.left.bumper.pressed) - Number($gamepadState.right.bumper.pressed)
                },
                angular: {
                    x: $gamepadState.right.stick.x,
                    y: $gamepadState.right.stick.y,
                    z: $gamepadState.left.trigger - $gamepadState.right.trigger
                }
            });
            robotMovementTopic.publish(twist);
        }
    };
*/
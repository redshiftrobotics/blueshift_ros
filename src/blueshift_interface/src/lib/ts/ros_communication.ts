import { browser } from '$app/env';

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
        var ros = new window.ROSLIB.Ros({
            url: `ws://${robotIP}:${websocketPort}`
        });

        ros.on('connection', function () {
            onconnection();
        });

        ros.on('error', function (error) {
            onerror(error);
        });

        ros.on('close', function () {
            onclose();
        });
        return ros
    }
}

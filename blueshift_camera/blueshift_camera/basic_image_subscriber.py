import asyncio
import threading
import time
import unittest

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.executors import ShutdownException
from rclpy.executors import SingleThreadedExecutor
from rclpy.task import Future

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from blueshift_interfaces.srv import WebRTCOfferCommunication
from rclpy.executors import SingleThreadedExecutor

import argparse
import asyncio
import json
import logging
import os
import platform
import ssl
import numpy
import math
import cv2
import time

from aiohttp import web

from aiortc import RTCPeerConnection, RTCSessionDescription
from aiortc.contrib.media import MediaPlayer, MediaRelay
from aiortc.rtcrtpsender import RTCRtpSender

from aiortc import (
    RTCIceCandidate,
    RTCPeerConnection,
    RTCSessionDescription,
    VideoStreamTrack,
)
from aiohttp import web
from av import VideoFrame
import threading



ROOT = os.path.dirname(__file__)

frames = asyncio.Event()
imgFrames = Image()
req = ""
res = ""
done = False
runOffer = False


class RosVideoStreamTrack(VideoStreamTrack):
    def __init__(self):
        super().__init__()  # don't forget this!
        self.br = CvBridge()
        print("rosvideostreamtrack init??")

    async def recv(self):
        print("rec function in rosvideostreamtrack")
        await frames.wait()
        global imgFrames
        current_frame = self.br.imgmsg_to_cv2(imgFrames)
        current_frame = VideoFrame.from_ndarray(current_frame, format="bgr24")

        frames.clear()
        return current_frame


def force_codec(pc, sender, forced_codec):
    kind = forced_codec.split("/")[0]
    codecs = RTCRtpSender.getCapabilities(kind).codecs
    transceiver = next(t for t in pc.getTransceivers() if t.sender == sender)
    transceiver.setCodecPreferences(
        [codec for codec in codecs if codec.mimeType == forced_codec]
    )


async def offer():
    global res
    global req
    global done
    requestSdp = req.sdp
    requestType = req.type

    offer = RTCSessionDescription(sdp=requestSdp, type=requestType)

    pc = RTCPeerConnection()
    pcs.add(pc)

    @pc.on("connectionstatechange")
    async def on_connectionstatechange():
        print("Connection state is %s" % pc.connectionState)
        if pc.connectionState == "failed":
            await pc.close()
            pcs.discard(pc)

    video_sender = pc.addTrack(
        MediaPlayer(
            "/dev/video0",
            format="v4l2",
            options={"framerate": "30", "video_size": "640x480"},
        ).video
        # RosVideoStreamTrack()
    )
    # force_codec(pc, video_sender, 'video/H264')

    await pc.setRemoteDescription(offer)

    await pc.setLocalDescription(await pc.createAnswer())

    res.sdp = pc.localDescription.sdp
    res.type = pc.localDescription.type
    done = True

    # while True:
    #     time.sleep(0.01)


def offerNotAsync(request, response):
    global req
    global res
    req = request
    res = response

    t1 = threading.Thread(target=asyncio.run, args=(offer(),))
    t1.start()
    t1.join()
    return res


pcs = set()


class ImageSubscriber(Node):
    """
    Create an ImageSubscriber class, which is a subclass of the Node class.
    """

    def __init__(self):
        """
        Class constructor to set up the node
        """
        # Initiate the Node class's constructor and give it a name
        super().__init__("image_subscriber")
        self.srv = self.create_service(
            WebRTCOfferCommunication, "web_RTC_offer_communication", self.test
        )

        self.subscription = self.create_subscription(
            Image, "video_frames", self.listener_callback, 10
        )
        self.subscription  # prevent unused variable warning

    def listener_callback(self, data):
        """
        Callback function.
        """
        # Display the message on the console
        # self.get_logger().info('Receiving video frame')
        global imgFrames
        imgFrames = data
        frames.set()

    def test(self, request, response):
        global req
        global res
        global runOffer
        req = request
        res = response

        # executor = SingleThreadedExecutor()
        # executor.add_node(self)

        # async def coroutine():
        #     await asyncio.sleep(5)
        #     return 'Sentinel Result'

        # future = executor.create_task(coroutine)
        # executor.spin_until_future_complete(future)
        # print(future.result())

        runOffer = True

        # gc = self.create_guard_condition(offer)
        # gc.trigger()
        # while not future.done():
        #     print(future.done())
        #     time.sleep(0.05)
        # self.destroy_guard_condition(gc)

        while not done:
            time.sleep(0.05)

        print(res)

        return res


def main(args=None):
    logging.basicConfig(level=logging.DEBUG)
    asyncio.run(mainAsync(args=args))

async def mainAsync(args=None):
    global runOffer

    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    image_subscriber = ImageSubscriber()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(image_subscriber)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    # Spin the node so the callback function is called.
    # rclpy.spin(image_subscriber)
    rate = image_subscriber.create_rate(10)

    try:
        while rclpy.ok():
            if runOffer:
                await offer()
                runOffer = False
            # rate.sleep()
            await asyncio.sleep(0.1)
    except KeyboardInterrupt:
        pass

    # image_subscriber.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()
    executor_thread.join()

    # Close all webrtc connections
    coros = [pc.close() for pc in pcs]
    await asyncio.gather(*coros)
    pcs.clear()


if __name__ == "__main__":
    main()

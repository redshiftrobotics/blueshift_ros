# Basic ROS 2 program to subscribe to real-time streaming 
# video from your built-in webcam
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com
   
# Import the necessary libraries
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
from blueshift_interfaces.srv import WebRTCOfferCommunication

# _______________________________________________________________________________
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
req = ''
res = ''

class RosVideoStreamTrack(VideoStreamTrack):


    def __init__(self):
        super().__init__()  # don't forget this!
        self.br = CvBridge()
        print('rosvideostreamtrack init??')

    async def recv(self):
        print('rec function in rosvideostreamtrack')
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
    requestSdp = req.sdp
    requestType = req.type
    print(requestSdp, requestType)
    
    offer = RTCSessionDescription(sdp=requestSdp, type=requestType)

    pc = RTCPeerConnection()
    pcs.add(pc)

    @pc.on("connectionstatechange")
    async def on_connectionstatechange():
        print("Connection state is %s" % pc.connectionState)
        if pc.connectionState == "failed":
            await pc.close()
            pcs.discard(pc)

    video_sender = pc.addTrack(MediaPlayer("/dev/video0", format="v4l2", options={"framerate": "30", "video_size": "640x480"}).video)
    # force_codec(pc, video_sender, 'video/H264')

    await pc.setRemoteDescription(offer)

    answer = await pc.createAnswer()
    await pc.setLocalDescription(answer)


    res.sdp = pc.localDescription.sdp
    res.type = pc.localDescription.type



def offerNotAsync(request, response):
    global req
    global res
    req = request
    res = response

    t1 = threading.Thread(target=asyncio.run, args=(offer(), ))
    t1.start()
    t1.join()
    return res


pcs = set()

async def on_shutdown(app):
    # close peer connections
    coros = [pc.close() for pc in pcs]
    await asyncio.gather(*coros)
    pcs.clear()

class ImageSubscriber(Node):
  """
  Create an ImageSubscriber class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('image_subscriber')
    self.srv = self.create_service(WebRTCOfferCommunication, 'web_RTC_offer_communication', offerNotAsync)

    self.subscription = self.create_subscription(
      Image, 
      'video_frames', 
      self.listener_callback, 
      10)
    self.subscription # prevent unused variable warning

  def listener_callback(self, data):
    """
    Callback function.
    """
    # Display the message on the console
    # self.get_logger().info('Receiving video frame')
    global imgFrames
    imgFrames = data
    frames.set()
   

def main(args=None):
  logging.basicConfig(level=logging.DEBUG)
   
  # Initialize the rclpy library
  rclpy.init(args=args)

  # Create the node
  image_subscriber = ImageSubscriber()

  # Spin the node so the callback function is called.
  rclpy.spin(image_subscriber)
 
  image_subscriber.destroy_node()
   
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
   
if __name__ == '__main__':
  main()
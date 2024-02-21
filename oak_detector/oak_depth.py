#!/usr/bin/env python3

import json
import sys
import rospy
import numpy as np
import depthai as dai
from sensor_msgs.msg import LaserScan

def publish_depth(depth_data):
    """
    Function to publish the depth along the middle line in the depth map
    """
    header = rospy.Header()
    header.stamp = rospy.Time.now()
    
    # Calculate middle index of the depth map
    middle_index = depth_data.shape[0] // 2
    print("Depth data shape: ", depth_data.shape)
    print("Middle index: ", middle_index)
    
    # Get depth along the middle line
    depth_values = depth_data[middle_index, :]
    print("Depth values: ", depth_values)
    
    # Create LaserScan message
    scan_msg = LaserScan()
    scan_msg.header = header
    scan_msg.angle_min = 0.0
    scan_msg.angle_max = 0.0
    scan_msg.angle_increment = 0.0
    scan_msg.time_increment = 0.0
    scan_msg.scan_time = 0.0
    scan_msg.range_min = 0.0
    scan_msg.range_max = 10.0  # Maximum range of 10 meters
    scan_msg.ranges = depth_values.tolist()
    
    pub_depth.publish(scan_msg)

def start_oak_camera():
    # Define camera pipeline and its components
    pipeline = dai.Pipeline()

    # Define needed components
    monoLeft = pipeline.create(dai.node.MonoCamera)
    monoRight = pipeline.create(dai.node.MonoCamera)
    stereo = pipeline.create(dai.node.StereoDepth)
    xoutDepth = pipeline.create(dai.node.XLinkOut)

    xoutDepth.setStreamName("depth")

    # Setting resolution for depth
    monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)

    stereo.setDepthAlign(dai.CameraBoardSocket.RGB)
    stereo.setOutputSize(640, 400)

    monoLeft.out.link(stereo.left)
    monoRight.out.link(stereo.right)
    stereo.depth.link(xoutDepth.input)

    # Connect to device and start processing
    with dai.Device(pipeline) as device:
        depthQueue = device.getOutputQueue(name="depth", maxSize=4, blocking=False)

        while True:
            depth_data = depthQueue.get().getFrame()
            publish_depth(depth_data)

if __name__ == "__main__":
    rospy.init_node("oak_depth_publisher", anonymous=True)
    pub_depth = rospy.Publisher("oak_depth", LaserScan, queue_size=1)
    start_oak_camera()

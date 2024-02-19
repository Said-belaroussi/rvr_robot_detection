#!/usr/bin/env python3

import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import depthai as dai

def publisher_images(frame):
    """
    Function to publish the frame as ROS messages
    """
    bridge = CvBridge()
    if frame is None:
        return
    frame_msg = bridge.cv2_to_imgmsg(frame, "bgr8")
    pub_frame.publish(frame_msg)

def start_oak_camera():
    """
    Function to start capturing frames from the OAK camera
    """
    # Define camera pipeline and its components
    pipeline = dai.Pipeline()

    # Define needed components
    camRgb = pipeline.create(dai.node.ColorCamera)
    xoutRgb = pipeline.create(dai.node.XLinkOut)

    xoutRgb.setStreamName("rgb")

    # Camera setup
    camRgb.setPreviewSize(1280, 720)  # Set resolution to 720p
    camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)

    camRgb.preview.link(xoutRgb.input)

    # Connect to device and start processing
    with dai.Device(pipeline) as device:
        # Output queue will be used to get the rgb frames
        queue = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
        running = True

        while running:
            try:
                inRgb = queue.get()  # Blocking call, will wait until a new data has arrived
                frame = inRgb.getCvFrame()  # Get OpenCV frame from NV12 data

                # Publish the frame
                publisher_images(frame)

            except KeyboardInterrupt:
                rospy.loginfo("Keyboard Interrupt detected. Stopping...")
                # Release resources and stop the camera
                device.close()
                running = False

if __name__ == "__main__":
    # Initialize ROS node and publishers
    rospy.init_node("oak_camera_publisher", anonymous=True)
    pub_frame = rospy.Publisher("oak_frames", Image, queue_size=1)

    start_oak_camera()

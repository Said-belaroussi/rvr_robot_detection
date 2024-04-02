import rospy
from std_msgs.msg import Header
from vision_msgs.msg import Detection2DArray
from vision_msgs.msg import Detection2D
from vision_msgs.msg import BoundingBox2D
import rospkg

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def publisher_bbox(bbox_data):
    """
    Function to publish the bounding box information as a Detection2DArray message
    """
    msg = Detection2DArray()
    header = Header()
    header.stamp = rospy.Time.now()
    msg.header = header
    detection_2d_list = []

    detection_2d_msg = Detection2D()
    detection_2d_msg.header = header
    bounding_box_2d = BoundingBox2D()
    bounding_box_2d.center.x = bbox_data["x"]
    bounding_box_2d.center.y = bbox_data["y"]
    bounding_box_2d.size_x = bbox_data["width"]
    bounding_box_2d.size_y = bbox_data["height"]
    detection_2d_msg.bbox = bounding_box_2d
    detection_2d_list.append(detection_2d_msg)

    msg.detections = detection_2d_list
    pub_bbox.publish(msg)

def start_oak_camera(blob_filename, json_filename):

    # Set up ROS publisher for bounding boxes
    rospy.init_node("oak_detector", anonymous=True)
    pub_bbox = rospy.Publisher("oak", Detection2DArray, queue_size=1)

    # Load configuration parameters from JSON file
    confidenceThreshold, numClasses, anchors, anchorMasks, \
        coordinateSize, iouThreshold, inputSizeX, inputSizeY, labelMap = get_config_from_json(json_filename)

    # Define camera pipeline and its components
    pipeline = dai.Pipeline()

    # Define needed components (replace YOLO spatial detection network with a standard object detection network)
    camRgb = pipeline.create(dai.node.ColorCamera)
    object_detection_network = pipeline.create(dai.node.MobileNetDetectionNetwork)
    object_detection_network.setConfidenceThreshold(confidenceThreshold)
    object_detection_network.setBoundingBoxScaleFactor(0.5)
    object_detection_network.setBlobPath(blob_filename)
    object_detection_network.setNumClasses(numClasses)

    camRgb.preview.link(object_detection_network.input)

    xout = pipeline.create(dai.node.XLinkOut)
    xout.setStreamName("detections")
    object_detection_network.out.link(xout.input)

    # Connect to device and start processing
    with dai.Device(pipeline) as device:
        outputQueue = device.getOutputQueue(name="detections", maxSize=4, blocking=False)

        while True:
            detections = outputQueue.get().detections

            for detection in detections:
                bbox_data = {
                    "x": detection.xmin,
                    "y": detection.ymin,
                    "width": detection.xmax - detection.xmin,
                    "height": detection.ymax - detection.ymin
                }

                publisher_bbox(bbox_data)  # Publish the bounding box data

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("usage: /path/oak_detector.py /path/blob_file_name /path/json_file_name")
        exit(1)
    else:
        blob_filename = sys.argv[1]
        json_filename = sys.argv[2]

    start_oak_camera(blob_filename, json_filename)

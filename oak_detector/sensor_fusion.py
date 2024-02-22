#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
import numpy as np
from filterpy.kalman import ExtendedKalmanFilter

class LaserFusionNode:
    def __init__(self):
        rospy.init_node('laser_fusion_node', anonymous=True)
        
        # Subscribe to laser scan topics
        rospy.Subscriber('/lidar_scan', LaserScan, self.lidar_callback)
        rospy.Subscriber('/camera_scan', LaserScan, self.camera_callback)
        
        # Create publisher for fused data
        self.fused_pub = rospy.Publisher('/fused_scan', LaserScan, queue_size=10)
        
        # Initialize Kalman Filter
        self.state_dim = 0  # State dimension
        self.measurement_dim = 0  # Measurement dimension
        
        # Define angle resolutions
        self.lidar_resolution = 0.43  # in degrees
        self.camera_resolution = 0.27  # in degrees
        
    def lidar_callback(self, data):
        # Process lidar scan data
        angles = np.arange(data.angle_min, data.angle_max, data.angle_increment)
        self.process_scan(data.ranges, angles, self.lidar_resolution)
        
    def camera_callback(self, data):
        # Process camera scan data
        angles = np.arange(data.angle_min, data.angle_max, data.angle_increment)
        self.process_scan(data.ranges, angles, self.camera_resolution)
        
    def process_scan(self, ranges, angles, resolution):
        # Fuse the data using Extended Kalman Filter
        measurements = np.array(ranges)
        angles = np.array(angles)
        
        # Convert polar coordinates to Cartesian coordinates
        x_coords = measurements * np.cos(angles)
        y_coords = measurements * np.sin(angles)
        
        # Stack x, y coordinates
        stacked_data = np.column_stack((x_coords, y_coords))
        
        # Update state and measurement dimensions
        self.state_dim = 2 * len(ranges)
        self.measurement_dim = len(stacked_data.flatten())
        
        # Initialize Kalman Filter if not initialized yet
        if not hasattr(self, 'ekf'):
            self.ekf = ExtendedKalmanFilter(dim_x=self.state_dim, dim_z=self.measurement_dim)
            self.ekf.x = np.zeros((self.state_dim, 1))  # Initial state
            self.ekf.P = np.eye(self.state_dim) * 0.01  # Initial covariance
        
        # Predict
        self.ekf.predict()
        
        # Update
        self.ekf.update(stacked_data.flatten())
        
        # Publish fused data
        fused_scan = LaserScan()
        fused_scan.header = Header()
        fused_scan.header.stamp = rospy.Time.now()
        fused_scan.header.frame_id = "fused_scan_frame"
        fused_scan.angle_min = -np.pi
        fused_scan.angle_max = np.pi
        fused_scan.angle_increment = resolution * np.pi / 180.0
        fused_scan.time_increment = 0.0
        fused_scan.scan_time = 0.0
        fused_scan.range_min = 0.0
        fused_scan.range_max = 100.0  # Modify
        fused_scan.ranges = self.ekf.x[::2].flatten().tolist()  # Extract x coordinates from the state
        self.fused_pub.publish(fused_scan)
        
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = LaserFusionNode()
        node.run()
    except rospy.ROSInterruptException:
        pass

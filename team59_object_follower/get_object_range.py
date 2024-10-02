#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
import numpy as np
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

class GetObjectRange(Node):
    def __init__(self):
        super().__init__('get_object_range')

        #Set up QoS Profiles for passing images over WiFi
        image_qos_profile = QoSProfile(depth=5)
        image_qos_profile.history = QoSHistoryPolicy.KEEP_LAST
        image_qos_profile.durability = QoSDurabilityPolicy.VOLATILE 
        image_qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT 


        # Subscriber to combined object location from detect_object node
        self.object_sub = self.create_subscription(Point, '/combined_object_location', self.object_callback, 10)

        # Subscriber to LIDAR scan data
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, image_qos_profile)

        # Publisher for object range (distance and angle)
        self.range_pub = self.create_publisher(Point, '/object_range', 10)

        # Store the latest LIDAR scan data
        self.lidar_data = None

        # Store the latest object location
        self.object_location = None

    def lidar_callback(self, data):
        """Callback for processing the LIDAR scan data."""
        # Filter out NaN values from the LIDAR ranges
        ranges_filtered = [r if not np.isnan(r) else 0.0 for r in data.ranges]
        data.ranges = ranges_filtered

        self.lidar_data = data

    def object_callback(self, data):
        """Callback for processing the object location."""
        self.object_location = data

        if self.lidar_data:
            self.compute_object_range()

    def compute_object_range(self):
        """Computes the object's range and angle based on LIDAR and camera data."""
        # Get the angle of the object from the object_location
        object_angle = np.arctan2(self.object_location.y, self.object_location.x)

        # Get the corresponding LIDAR distance for the object angle
        lidar_angle_index = int((object_angle + np.pi) / self.lidar_data.angle_increment)
        lidar_distance = self.lidar_data.ranges[lidar_angle_index]
        lidar_distance = self.lidar_data.ranges[0.0]

        # Create Point message for object range (distance and angle)
        object_point = Point()
        object_point.x = lidar_distance  # Distance from robot to object
        object_point.y = object_angle    # Angle of the object relative to the robot
        object_point.z = 0.0  # Unused, can be used for height or other data

        # Publish the object's range
        self.range_pub.publish(object_point)


def main(args=None):
    rclpy.init(args=args)

    # Create the node
    get_object_range_node = GetObjectRange()

    # Spin to keep the node running
    rclpy.spin(get_object_range_node)

    # Cleanup when shutting down
    get_object_range_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
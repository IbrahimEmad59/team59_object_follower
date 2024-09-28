#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, LaserScan
from geometry_msgs.msg import Point
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy


class DetectObject(Node):
    def __init__(self):
        # Initialize ROS node
        super().__init__('detect_object')
        
        # Bridge for converting ROS image messages to OpenCV format
        self.bridge = CvBridge()

        #Set up QoS Profiles for passing images over WiFi
        image_qos_profile = QoSProfile(depth=5)
        image_qos_profile.history = QoSHistoryPolicy.KEEP_LAST
        image_qos_profile.durability = QoSDurabilityPolicy.VOLATILE 
        image_qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT 

        # Subscriber to compressed image topic from the camera
        self.image_sub = self.create_subscription(CompressedImage,"/image_raw/compressed", self.image_callback, image_qos_profile)

        # Subscriber to LIDAR scan topic
        self.lidar_sub = self.create_subscription(LaserScan, "/scan", self.lidar_callback, image_qos_profile)

        # Publisher for the detected object's location in combined reference frame
        self.object_pub = self.create_publisher(Point, "/combined_object_location", 10)

        # Store the latest LIDAR scan data
        self.lidar_data = None

    def image_callback(self, data):
        """Callback for processing the camera image data."""
        try:
            # Convert the compressed image to OpenCV format
            np_arr = np.frombuffer(data.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except CvBridgeError as e:
            self.get_logger().info(f"Error converting image: {e}")
            return

        # Process the image to detect the object (simple color thresholding for example)
        object_location = self.detect_object_in_image(cv_image)

        if object_location is not None and self.lidar_data is not None:
            # Combine camera and LIDAR data to get the object's location in a common reference frame
            combined_location = self.combine_camera_lidar(object_location)
            
            if combined_location:
                # Publish the combined object location
                self.object_pub.publish(combined_location)

    def lidar_callback(self, data):
        """Callback for processing the LIDAR scan data."""
        # Store the LIDAR data for later use
        self.lidar_data = data

    def detect_object_in_image(self, image):
        """Detect the object in the camera image."""
        # Convert the image to HSV (for color detection)
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define range for the object color in HSV (this depends on the object you're tracking)
        lower_color = np.array([0, 120, 70])
        upper_color = np.array([10, 255, 255])

        # Threshold the HSV image to get the object
        mask = cv2.inRange(hsv_image, lower_color, upper_color)

        # Find contours of the object
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # Get the largest contour (assuming it's the object)
            largest_contour = max(contours, key=cv2.contourArea)

            # Get the center of the contour
            M = cv2.moments(largest_contour)
            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                self.get_logger().info(f"Object detected at: ({cx}, {cy})")
                return (cx, cy)
        
        return None

    def combine_camera_lidar(self, object_location):
        """Combine camera and LIDAR data to determine the object's location."""
        cx, cy = object_location

        if self.lidar_data is None:
            return None

        # Assume that the camera's field of view matches the LIDAR's field of view
        # Map the object's pixel location to an angle for LIDAR data
        image_width = 320  # Image width in pixels 
        fov_degrees = 62.2  # Camera field of view 
        
        # Calculate the angle in radians based on the horizontal pixel position
        angle = (cx - image_width / 2.0) * (fov_degrees / image_width) * (np.pi / 180.0)
        self.get_logger().info(f"The current angle is: {angle}")

        # Now, find the corresponding LIDAR distance at that angle (this is a basic approach)
        lidar_angle_index = int((angle + np.pi) * len(self.lidar_data.ranges) / (2 * np.pi))
        lidar_distance = self.lidar_data.ranges[lidar_angle_index]

        # Create a Point message for the object's position in polar coordinates (angle, distance)
        object_point = Point()
        object_point.x = lidar_distance * np.cos(angle)
        object_point.y = lidar_distance * np.sin(angle)
        object_point.z = 0.0  # Assuming the object is on the same plane as the robot

        return object_point

def main():
    rclpy.init() #init routine needed for ROS2.
    # Create the detect_object node
    detect_object_node = DetectObject()
   
    rclpy.spin(detect_object_node) # Trigger callback processing.		

    #Clean up and shutdown.
    detect_object_node.destroy_node()  
    rclpy.shutdown()

if __name__ == '__main__':
    main()
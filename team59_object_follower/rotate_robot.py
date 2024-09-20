#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy



class RotatingNode(Node):

    def __init__(self):
        super().__init__("rotate_robot")
        #Set up QoS Profiles for passing images over WiFi

        image_qos_profile = QoSProfile(depth=5)
        image_qos_profile.history = QoSHistoryPolicy.KEEP_LAST
        image_qos_profile.durability = QoSDurabilityPolicy.VOLATILE 
        image_qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT 
        
        self.location = self.create_subscription(Point, '/geometry_msgs',self.rotate_callback,image_qos_profile)
        self.rotate_command_publisher = self.create_publisher(Twist,'/cmd_vel', 10)
    
    def rotate_callback(self, location: Point):
        #self.get_logger().info(str(location.x))
        msg = Twist()
        msg.angular.z = 0.0
        if(location.x - 155 > 20):
            msg.angular.z = -0.3
        elif(155 - location.x > 20):
            msg.angular.z = 0.3
        else:
            msg.angular.z = 0.0
        self.rotate_command_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RotatingNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()


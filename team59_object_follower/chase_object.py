#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
import math

class PIDController:
    def __init__(self, kp, ki, kd, output_limits=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0
        self.output_limits = output_limits  # Min/max limits for output (if any)

    def compute(self, error, dt):
        """Compute the control signal based on the error and time step."""
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error

        # Apply output limits if provided
        if self.output_limits is not None:
            min_output, max_output = self.output_limits
            output = max(min(output, max_output), min_output)

        return output

class ChaseObject(Node):
    def __init__(self):
        super().__init__('chase_object')

        # Maximum velocities (linear and angular)
        self.max_linear_velocity = 0.1  # meters per second
        self.max_angular_velocity = 1.5  # radians per second

        # Desired distance to the object (1 meter, adjust as needed)
        self.desired_distance = 0.5  # meter

        # 5% tolerance on distance and angle
        self.distance_tolerance = 0.05 * self.desired_distance  # 5% of the desired distance
        self.angle_tolerance = 0.05  # 5% tolerance in radians (adjust based on your application)

        # PID controllers for angular and linear control, with output limits
        self.angular_pid = PIDController(kp=1.2, ki=0.0, kd=0.5, output_limits=(-self.max_angular_velocity, self.max_angular_velocity))
        self.linear_pid = PIDController(kp=1.0, ki=0.0, kd=0.1, output_limits=(0.0, self.max_linear_velocity))

        # Subscriber to object range (distance and angle)
        self.range_sub = self.create_subscription(Point, '/object_range', self.range_callback, 10)

        # Publisher for velocity commands
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Time tracking for PID computation
        self.prev_time = self.get_clock().now()

    def range_callback(self, data):
        """Callback to process the object range data."""
        # Extract the distance and angle from the message
        object_distance = data.x
        object_angle = data.y

        # Get current time and compute time step
        current_time = self.get_clock().now()
        dt = (current_time - self.prev_time).nanoseconds / 1e9  # Convert nanoseconds to seconds
        self.prev_time = current_time

        # Compute angular error (we want the angle to be 0, i.e., facing the object)
        angular_error = object_angle

        # Compute linear error (we want the distance to be self.desired_distance)
        linear_error = self.desired_distance - object_distance

        # Check if the errors are within tolerance
        if abs(angular_error) < self.angle_tolerance:
            angular_velocity = 0.0  # No need to rotate further
        else:
            angular_velocity = self.angular_pid.compute(angular_error, dt)

        if abs(linear_error) < self.distance_tolerance:
            linear_velocity = 0.0  # No need to move forward/backward further
        else:
            linear_velocity = self.linear_pid.compute(linear_error, dt)

        # Ensure the computed velocities are within the max limits
        linear_velocity = max(min(linear_velocity, self.max_linear_velocity), 0.0)
        angular_velocity = max(min(angular_velocity, self.max_angular_velocity), -self.max_angular_velocity)

        self.get_logger().info(f"The linear velocity: {linear_velocity}")
        self.get_logger().info(f"The angular velocity: {angular_velocity}")

        # Create Twist message with linear and angular velocity
        twist = Twist()
        twist.linear.x = linear_velocity  # Forward/backward velocity
        twist.angular.z = angular_velocity  # Rotational velocity

        # Publish the velocity commands
        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)

    # Create the node
    chase_object_node = ChaseObject()

    # Spin to keep the node running
    rclpy.spin(chase_object_node)

    # Cleanup when shutting down
    chase_object_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
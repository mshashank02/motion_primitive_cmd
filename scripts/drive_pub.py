#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from phasespace_msgs.msg import Marker
from phasespace_msgs.msg import Markers
from numpy import load

class StraightLineDriver(Node):
    def __init__(self):
        super().__init__('straight_line_driver')

        self.publisher = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.timer = self.create_timer(0.1, self.publish_straight_line_command)
        self.duration = 1.0  # Set the duration for straight-line driving in seconds
        self.start_time = self.get_clock().now().to_msg()
        

    def publish_straight_line_command(self):

        current_time = self.get_clock().now().to_msg()
        elapsed_time = (current_time.sec - self.start_time.sec) + (current_time.nanosec - self.start_time.nanosec) / 1e9

        if elapsed_time < self.duration:
            # Create an AckermannDriveStamped message
            drive_msg = AckermannDriveStamped()

            # Set the header with ROS time
            drive_msg.header.stamp = current_time

            # Set the linear speed for straight-line driving
            drive_msg.drive.speed = 0.8  # Adjust the speed as needed

            # Set the steering angle (0 for straight-line driving)
            drive_msg.drive.steering_angle = 0.0

            # Publish the command to the /drive topic
            self.publisher.publish(drive_msg)
        else:
            # Stop the car by publishing a command with zero speed
            stop_msg = AckermannDriveStamped()
            stop_msg.header.stamp = current_time
            stop_msg.drive.speed = 0.0
            self.publisher.publish(stop_msg)

 

def main(args=None):
    rclpy.init(args=args)
    straight_line_driver = StraightLineDriver()
    rclpy.spin(straight_line_driver)
    straight_line_driver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


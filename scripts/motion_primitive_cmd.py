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

        self.get_waypoints()
        #Subscriber for markers
        self.markers_subscriber = self.create_subscription(Markers,'/synchronized_markers',self.get_current_states,10)
        self.markers_subscriber


        #self.publisher = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        #self.duration = 1000000000000000.0  # Set the duration for straight-line driving in seconds
        #self.start_time = self.get_clock().now().to_msg()
        
    def get_waypoints(self): 

        data = np.load('/home/admin1/f1tenthros2_ws/src/motion_primitive_cmd/waypoints/waypoints_data.npz')
        self.waypoints = data['waypoints']
        print(self.waypoints)
    
        
    def get_current_states(self,marker_msg):

        #Get the current marker array 
        marker_array = marker_msg.markers

        #Get individual marker position
        for marker in marker_array:

            if(marker.id == 0):
                current_x1 = marker.x
                current_y1 = marker.y
            elif(marker.id == 1):
                current_x2 = marker.x
                current_y2 = marker.y
            else:
                continue
        
        current_x = (current_x1 + current_x2)/2
        current_y = (current_y1 + current_y2)/2
        self.current_position = np.array([current_x,current_y])
        print(self.current_position)
        self.current_yaw = np.arctan((current_y2-current_y1)/(current_x2-current_x1))
        print(self.current_yaw)


    def find_target_waypoint(self):

        #Calculate distance to each waypoint
        distances = [np.linalg.norm(np.array(waypoint) - self.current_position) for waypoint in self.waypoints]
        #Index of closest waypoint 
        closest_index = np.argmin(distances)
        #Closest waypoint to current position
        self.target_waypoint = self.waypoints[closest_index]
        print(self.target_waypoint)
    
    def angle_controller(self):

        #Proportional gain for yaw to steering angle 
        Kp = 1
        #Find the error in position 
        error = self.target_waypoint - self.current_position
        #Find the target yaw
        target_yaw = np.arctan(error[1]/error[0])
        #Find error in yaw
        error_yaw = target_yaw - self.current_yaw
        #Control command for steering angle 
        self.steering_command = Kp*error_yaw

        #Clipping for max and min values 
        if(abs(self.steering_command)>0.85):
            self.steering_command = 0.85
            print("Clipping steering angle to 0.85")
        elif(abs(self.steering_command)<0.15):
            self.steering_command = 0.15
            print("Clipping steering angle to 0.15")
    

    def publish_drive_command(self):

        current_time = self.get_clock().now().to_msg()
        elapsed_time = (current_time.sec - self.start_time.sec) + (current_time.nanosec - self.start_time.nanosec) / 1e9

        if elapsed_time < self.duration:
            # Create an AckermannDriveStamped message
            drive_msg = AckermannDriveStamped()
            
            # Set the linear speed for straight-line driving
            drive_msg.drive.speed = 0.8  # Adjust the speed as needed

            # Set the steering angle (0 for straight-line driving)
            drive_msg.drive.steering_angle = self.steering_command

            # Publish the command to the /drive topic
            self.publisher.publish(drive_msg)
        else:
            # Stop the car by publishing a command with zero speed
            stop_msg = AckermannDriveStamped()
            stop_msg.drive.speed = 0.0
            self.publisher.publish(stop_msg)

    def timer_callback(self):

        #Get current state of the robot 
        #self.get_current_states()

        #Find the closest target waypoint
        self.find_target_waypoint()

        #Run angle controller to find the correct steering angle 
        #self.angle_controller()

        #Publish velocity and steering angle commands
        #self.publish_drive_command()

def main(args=None):
    rclpy.init(args=args)
    straight_line_driver = StraightLineDriver()
    rclpy.spin(straight_line_driver)
    straight_line_driver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


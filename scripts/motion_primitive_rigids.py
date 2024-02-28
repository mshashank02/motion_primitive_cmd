#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from phasespace_msgs.msg import Rigid
from phasespace_msgs.msg import Rigids
import math
import quaternion 

class StraightLineDriver(Node):
    def __init__(self):
        super().__init__('straight_line_driver')

        self.get_waypoints()
        #Subscriber for markers
        self.markers_subscriber = self.create_subscription(Rigids,'/synchronized_rigids',self.get_current_states,10)
        self.markers_subscriber

	#Publisher for velocity commands
        self.publisher = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.duration = 1000000000000000.0  # Set the duration for driving in seconds
        self.start_time = self.get_clock().now().to_msg()
        
    def get_waypoints(self): 

        data = np.load('/home/admin1/f1tenthros2_ws/src/motion_primitive_cmd/waypoints/waypoints_data.npz')
        self.waypoints = data['waypoints']
        print(self.waypoints)
            
        
    def get_current_states(self,rigids_msg):

        #Get the current marker array 
        rigids_array = rigids_msg.msg

        #Get individual marker position
        for rigid in rigids_array:

            current_x = rigid.x
            current_y = rigid.y
            current_qw = rigid.qw
            current_qx = rigid.qx
            current_qy = rigid.qy
            current_qz = rigid.qz

    
        self.current_position = np.array([current_x,current_y])
        print(f"current position is: {self.current_position}")
        self.current_quat = np.qauaternion(current_qw,current_qx,current_qy,current_qz)
        print(f"current yaw is: {self.current_quat}")

        


    def find_target_waypoint(self):

        #Calculate distance to each waypoint
        distances = [np.linalg.norm(np.array(waypoint) - self.current_position) for waypoint in self.waypoints]
        
        #Index of closest waypoint 
        closest_index = np.argmin(distances)
        
        #Closest waypoint to current position
        self.target_waypoint = self.waypoints[closest_index]
        
        print(f"target waypoint is: {self.target_waypoint}")
        
    
    def angle_controller(self):

        #Proportional gain for yaw to steering angle 
        Kp = 0.1
        #Find the error in position 
        error = self.target_waypoint - self.current_position
        #Find the target yaw
        target_yaw = math.atan2(error[1],error[0])
        print(f"target yaw is: {target_yaw}")
        #Find error in yaw
        error_yaw = target_yaw - self.current_yaw
        print(f"error in yaw: {error_yaw}")
        #Control command for steering angle 
        self.steering_command = Kp*error_yaw
        print(f"steering angle setpoint: {self.steering_command}")

        #Clipping for max and min values 
        if(self.steering_command <= -0.4494):
            self.steering_command = -0.4494
            print("Clipping steering angle to -0.4494")
        elif(self.steering_command >= 0.3949):
            self.steering_command = 0.3949
            print("Clipping steering angle to 0.3949")
    
    

    

    def publish_drive_command(self):

        current_time = self.get_clock().now().to_msg()
        elapsed_time = (current_time.sec - self.start_time.sec) + (current_time.nanosec - self.start_time.nanosec) / 1e9

        if elapsed_time < self.duration:
            # Create an AckermannDriveStamped message
            drive_msg = AckermannDriveStamped()
            
            drive_msg.header.stamp = current_time
            # Set the linear speed for straight-line driving
            drive_msg.drive.speed = 0.8  # Adjust the speed as needed

            # Set the steering angle (0 for straight-line driving)
            drive_msg.drive.steering_angle = self.steering_command

            # Publish the command to the /drive topic
            self.publisher.publish(drive_msg)
        else:
            # Stop the car by publishing a command with zero speed
            stop_msg = AckermannDriveStamped()
            stop_msg.header.stamp = current_time
            stop_msg.drive.speed = 0.0
            self.publisher.publish(stop_msg)

    def timer_callback(self):


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


#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from phasespace_msgs.msg import Rigid
from phasespace_msgs.msg import Rigids
import math
import quaternion 
import time

class StraightLineDriver(Node):
    def __init__(self):
        super().__init__('straight_line_driver')

        self.get_waypoints()

        self.current_position = None
        self.initial_position_set = False
        self.reorder_called = False
        
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
        rigids_array = rigids_msg.rigids

        #Get individual marker position
        for rigid in rigids_array:
            
            if(rigid.id ==1):

                self.current_position = np.array([rigid.x,rigid.y])
                if not self.initial_position_set:
                    self.reorder_waypoints()
                    self.initial_position_set = True
                print(f"current position is: {self.current_position}")
                current_quat = np.quaternion(rigid.qw,rigid.qx,rigid.qy,rigid.qz)
                print(f"current quaternion is: {current_quat}")
                current_rot = quaternion.as_rotation_matrix(current_quat)
                print(f"current rotation matrix is: {current_rot}")
                self.yaw = np.arctan2(current_rot[1, 0], current_rot[0, 0])
                print(f"current yaw is: {self.yaw}")
                break

    

    def reorder_waypoints(self):

        if not self.reorder_called:


            #Calculate distance to each waypoint
            distances = [np.linalg.norm(np.array(waypoint) - self.current_position) for waypoint in self.waypoints]
        
            #Index of closest waypoint 
            closest_index = np.argmin(distances)
        
            #Reorder waypoints starting from the closest point 
            self.reordered_waypoints = np.roll(self.waypoints,-closest_index,axis=0)

            self.reorder_called = True
        
    
    def waypoint_reached(self):
        
        i = 0
           
        #Looping through the reordered waypoints   
        while (i <= np.size(self.reordered_waypoints)):

            self.target_waypoint = self.reordered_waypoints[i] 
            distance_to_waypoint = np.linalg.norm(self.target_waypoint - self.current_position)
            min_distance = 0.01  #TODO Tune this parameter
            if(distance_to_waypoint <= min_distance):
                print("Reached {i}th waypoint")
                i += 1

        
        
        
    
    def angle_controller(self):

        #Proportional gain for yaw to steering angle 
        #TODO Tune this parameter 
        Kp = 0.1
        #Find the error in position 
        error = self.target_waypoint - self.current_position
        #Find the target yaw
        target_yaw = np.arctan2(error[1],error[0])
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


        #Find distance to waypoint
        self.waypoint_reached()

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


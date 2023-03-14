#!/usr/bin/env python

import math
import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Point, Twist, Pose
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

ANG_TOL = (5*math.pi)/180 # 5 deg
LIN_TOL = 0.30 # 30 cm
LIN_SETPT = 0.30
ANG_SETPT = 0.0
TOL = 1e-1

class RotateRobot(Node):

    def __init__(self):

        super().__init__("rotate_robot")

        self.declare_parameter("cmd_vel_topic","/cmd_vel") 
        self.declare_parameter("kp", 0.5) 

        self.kp = self.get_parameter("kp").value
        
        # Subscriber to the track coordinates
        self.ang_subs = self.create_subscription(Float64, 
                                                "/angle_wrt_lidar", 
                                                self.callback_ang_coords,
                                                1)
        # Publisher to the command velocity
        self.cmd_vel_pubs = self.create_publisher(Twist, 
                                                  self.get_parameter("cmd_vel_topic").value,
                                                  10)

        #Subscriber to object distance
        self.dist_subs = self.create_subscription(Float64,
                                                  "/object_dist",
                                                  self.callback_dist_coords,
                                                  10)


        # Command Velocity Publisher Timer
        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.current_twist = Twist()
        self.current_twist.linear.x = 0.0
        self.current_twist.linear.y = 0.0
        self.current_twist.linear.z = 0.0
        self.current_twist.angular.x = 0.0
        self.current_twist.angular.y = 0.0
        self.current_twist.angular.z = 0.0

        # PID
        self.kp_ang = 1.0
        self.kd_ang = 0.1
        self.kp_lin = 1.00
        self.kd_lin = 0.5

        self.current_angle = None
        self.current_dist = None
        self.prev_angle = None
        self.prev_dist = None

        # Flag for angular and linear
        self.ang_flag = False
        self.lin_flag = False

    def callback_dist_coords(self, msg):
        self.current_dist = msg.data
        self.dist_err = self.current_dist - LIN_SETPT
        if abs(self.dist_err) < TOL:
            print("Linear Tolerance Satisfied")
            self.lin_flag = True
        else:
            self.lin_flag = False 
        # if self.current_dist == LIN_TOL:
        #     print("Linear Tolerance Satisfied")
        #     self.lin_flag = True
        # else:
        #     self.lin_flag = False 


    def callback_ang_coords(self, msg):
        self.current_angle = msg.data
        self.ang_err = self.current_angle - ANG_SETPT
        if abs(self.ang_err) < TOL:
            print("Angular Tolerance Satisfied")
            self.ang_flag = True
        else:
            self.ang_flag = False
        # if -ANG_TOL <= self.current_angle <= ANG_TOL:
        #     print("Angular Tolerance Satisfied")
        #     self.ang_flag = True
        # else:
        #     self.ang_flag = False
    
    # def timer_callback(self):
    #     print(f"The Lin flag : {self.lin_flag} and Ang Flag : {self.ang_flag}")
    #     if self.current_angle is not None and self.current_dist is not None:
    #         if self.prev_angle is not None and self.prev_dist is not None:
    #             if self.ang_flag:
    #                 if not self.lin_flag:
    #                     if self.current_dist > LIN_TOL:
    #                         print("Move Forward")
    #                         self.current_twist.linear.x = self.kp_lin * self.current_dist #+ self.kd_lin * (self.current_dist - self.prev_dist) / 0.1
    #                         self.current_twist.angular.z = 0.0
    #                     elif self.current_dist < LIN_TOL:
    #                         print("Move Backward")
    #                         self.current_twist.linear.x = -self.kp_lin * self.current_dist #+ self.kd_lin * (self.current_dist - self.prev_dist) / 0.1
    #                         self.current_twist.angular.z = 0.0
    #                 else:
    #                     print("REST")
    #                     self.current_twist.linear.x = 0.0
    #                     self.current_twist.angular.z = 0.0
    #             elif not self.ang_flag:
    #                 print("Rotating")
    #                 self.current_twist.angular.z = self.kp_ang * self.current_angle #+ self.kd_ang * (self.current_angle - self.prev_angle) / 0.1
    #                 self.current_twist.linear.x = 0.0
    #             else:
    #                 print("REST")
    #                 self.current_twist.linear.x = 0.0
    #                 self.current_twist.angular.z = 0.05
    #         else:
    #             self.prev_angle = self.current_angle
    #             self.prev_dist = self.current_dist


    #         self.cmd_vel_pubs.publish(self.current_twist)

    def timer_callback(self):
        print(f"The Lin flag : {self.lin_flag} and Ang Flag : {self.ang_flag}")
        if self.current_angle is not None and self.current_dist is not None:
            if self.ang_flag:
                if not self.lin_flag:
                    print("LINEAR")
                    self.current_twist.linear.x = self.kp_lin * self.dist_err #+ self.kd_lin * (self.current_dist - self.prev_dist) / 0.1
                    self.current_twist.angular.z = 0.0      
                else:
                    print("REST")
                    self.current_twist.linear.x = 0.0
                    self.current_twist.angular.z = 0.0
            elif not self.ang_flag:
                print("Rotating")
                self.current_twist.angular.z = self.kp_ang * self.ang_err #+ self.kd_ang * (self.current_angle - self.prev_angle) / 0.1
                self.current_twist.linear.x = 0.0
            else:
                print("REST")
                self.current_twist.linear.x = 0.0
                self.current_twist.angular.z = 0.05


        self.cmd_vel_pubs.publish(self.current_twist)


def main():
    rclpy.init() #init routine needed for ROS2.
    rotate_robot = RotateRobot()

    rclpy.spin(rotate_robot)

    #Clean up and shutdown.
    rotate_robot.destroy_node()  
    rclpy.shutdown()


if __name__ == '__main__':
    main()
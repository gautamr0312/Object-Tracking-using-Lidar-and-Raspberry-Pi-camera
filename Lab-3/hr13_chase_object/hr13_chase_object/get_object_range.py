#!/usr/bin/env python

import rclpy
from math import pi
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy, qos_profile_sensor_data

PI = pi

class LidarNode(Node):

    def __init__(self):

        print("Lidar Node Started....")

        super().__init__("object_range")
        
        self.declare_parameter("lidar_threshold", 5)


        self.lidar_subs = self.create_subscription(LaserScan,
                                                   "/scan",
                                                   self.callback_lidar_range,
                                                   qos_profile_sensor_data)

        self.lidar_subs

        self.dist_pubs = self.create_publisher(Float64, "/object_dist", 10)

        self.angle_subs = self.create_subscription(Float64,
                                                   "/angle_wrt_lidar",
                                                   self.callback_angle_lidar,
                                                   1)

        self.angle_wrt_lidar = None

    def callback_angle_lidar(self, msg):
        print("Callback angle Lidar...")
        print("Angle wrt Lidar Received...")
        self.angle_wrt_lidar = msg.data

    def callback_lidar_range(self, msg):
        print("Callback Lidar...")
        if self.angle_wrt_lidar is not None:
            angle_min = msg.angle_min
            angle_max = msg.angle_max
            angle_inc = msg.angle_increment
            len_rng_arr = len(msg.ranges)
            # ang_inc = msg.angle_increment
            print(f"Length of array check : {round((angle_max - angle_min ) / angle_inc) == len_rng_arr}")

            index = round( (abs(self.angle_wrt_lidar) - angle_min ) / angle_inc)
            # print(f'The index of lidar is {index}')

            index_min = round(index - self.get_parameter("lidar_threshold").value)
            index_max = round(index + self.get_parameter("lidar_threshold").value)

            if index_min < 0 :
                index_min = len_rng_arr + index_min
                rng_arr = np.array(msg.ranges[index_min:len_rng_arr]+msg.ranges[0:index_max])

            elif index_max > len_rng_arr:
                index_max = index_max - len_rng_arr
                rng_arr = np.array(msg.ranges[index_min:len_rng_arr]+msg.ranges[0:(index_max-len_rng_arr)])
                
            else:
                rng_arr = np.array(msg.ranges[index_min:index_max])
                

            rng_arr =rng_arr[~np.isnan(rng_arr)]
            dist = float(np.mean(rng_arr))

            print(f"The range of lidar index : {index_min, index_max}")
            print(f"The distance from object is {dist}")

            msg = Float64()
            msg.data = dist
            self.dist_pubs.publish(msg)


def main():
    rclpy.init() #init routine needed for ROS2.
    lidar_node = LidarNode()

    rclpy.spin(lidar_node)

    #Clean up and shutdown.
    lidar_node.destroy_node()  
    rclpy.shutdown()


if __name__ == '__main__':
    main()







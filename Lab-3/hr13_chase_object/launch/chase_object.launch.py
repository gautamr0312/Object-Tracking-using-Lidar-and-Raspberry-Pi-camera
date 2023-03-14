#!/usr/bin/env python

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()

    package_name = 'hr13_chase_object'

    # config = os.path.join(
    #     get_package_share_directory('hr13_chase_object'),
    #     'config',
    #     'params.yaml')

    # sys_cam_node = Node(
    #     package=package_name,
    #     name='system_camera',
    #     executable='system_camera',
    # )

    detect_object = Node(
        package=package_name,
        name='detect_object',
        executable='detect_object',
        # output="screen"
    )

    chase_object = Node(
        package=package_name,
        name='chase_object',
        executable='chase_object',
        output="screen"
    )

    get_object_range = Node(
        package=package_name,
        name='get_object_range',
        executable='get_object_range',
        # output="screen"
    )

    ld.add_action(detect_object)
    ld.add_action(chase_object)
    ld.add_action(get_object_range)
    
    return ld
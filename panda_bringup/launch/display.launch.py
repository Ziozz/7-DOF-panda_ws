#!/usr/bin/env python3
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Build robot_description from xacro using the xacro executable
    pkg_share = FindPackageShare(package='panda_description')
    xacro_file = PathJoinSubstitution([pkg_share, 'urdf', 'panda.urdf.xacro'])
    robot_description_content = Command([FindExecutable(name='xacro'), ' ', xacro_file])

    # robot_state_publisher node
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content
        }]
    )

    # joint_state_publisher_gui (optional) to manipulate joints interactively
    jsp_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
    )

    # rviz2 (no config, user can add RobotModel display)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', '']
    )

    return LaunchDescription([
        rsp_node,
        jsp_node,
        rviz_node
    ])


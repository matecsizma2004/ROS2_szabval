"""Teljes ragasztási szimuláció: KUKA KR6 R900-2 mock hardverrel + MoveIt + saját node-ok."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('ragasztas')
    params = os.path.join(pkg, 'config', 'glue_map.yaml')

    # ros2_control (mock hardver) + robot_state_publisher + move_group + MoveIt RViz
    kuka = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('kuka_kr_moveit_config'),
            'launch', 'moveit_planning_fake_hardware.launch.py')),
        launch_arguments={'robot_model': 'kr6_r900_2', 'robot_family': 'agilus'}.items(),
    )

    sensor = Node(package='ragasztas', executable='glue_map_sensor',
                  parameters=[params], output='screen')

    # A move_group és a controllerek felállása után indul.
    motion = TimerAction(period=5.0, actions=[
        Node(package='ragasztas', executable='glue_motion',
             parameters=[params], output='screen')])

    rviz = Node(package='rviz2', executable='rviz2', name='ragasztas_rviz',
                arguments=['-d', os.path.join(pkg, 'rviz', 'ragasztas.rviz')],
                output='log')

    return LaunchDescription([kuka, sensor, rviz, motion])

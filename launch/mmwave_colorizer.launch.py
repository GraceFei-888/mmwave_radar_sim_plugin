"""
mmwave_colorizer.launch.py  (top-level convenience copy)

Launches the mmwave_cloud_colorizer node.
Delegates to the copy inside the mmwave_cloud_tools package.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('input_cloud',    default_value='/mmwave/points'),
        DeclareLaunchArgument('output_cloud',   default_value='/mmwave/points_colored'),
        DeclareLaunchArgument('velocity_field', default_value='v'),
        DeclareLaunchArgument('v_min',          default_value='-10.0'),
        DeclareLaunchArgument('v_max',          default_value='10.0'),
        DeclareLaunchArgument('colormap',       default_value='bwr'),
        DeclareLaunchArgument('clamp',          default_value='true'),

        Node(
            package='mmwave_cloud_tools',
            executable='mmwave_cloud_colorizer',
            name='mmwave_cloud_colorizer',
            output='screen',
            parameters=[{
                'input_cloud':    LaunchConfiguration('input_cloud'),
                'output_cloud':   LaunchConfiguration('output_cloud'),
                'velocity_field': LaunchConfiguration('velocity_field'),
                'v_min':          LaunchConfiguration('v_min'),
                'v_max':          LaunchConfiguration('v_max'),
                'colormap':       LaunchConfiguration('colormap'),
                'clamp':          LaunchConfiguration('clamp'),
            }],
        ),
    ])

"""
mmwave_colorizer.launch.py

Launches the mmwave_cloud_colorizer node.

Override velocity range to match your SDF configuration, e.g.:
  ros2 launch mmwave_cloud_tools mmwave_colorizer.launch.py v_min:=-5.0 v_max:=5.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # ── Arguments ────────────────────────────────────────────────────────
        DeclareLaunchArgument(
            'input_cloud',
            default_value='/mmwave/points',
            description='Input PointCloud2 topic (must contain a float32 "v" field).'
        ),
        DeclareLaunchArgument(
            'output_cloud',
            default_value='/mmwave/points_colored',
            description='Output PointCloud2 topic (adds "rgb" field for RViz2).'
        ),
        DeclareLaunchArgument(
            'velocity_field',
            default_value='v',
            description='Name of the radial-velocity field in the input cloud.'
        ),
        DeclareLaunchArgument(
            'v_min',
            default_value='-10.0',
            description='Velocity (m/s) mapped to blue.  Should match <v_min> in SDF.'
        ),
        DeclareLaunchArgument(
            'v_max',
            default_value='10.0',
            description='Velocity (m/s) mapped to red.  Should match <v_max> in SDF.'
        ),
        DeclareLaunchArgument(
            'colormap',
            default_value='bwr',
            description='Colour map name.  Currently only "bwr" (blue-white-red) is supported.'
        ),
        DeclareLaunchArgument(
            'clamp',
            default_value='true',
            description='Clamp velocities outside [v_min, v_max] to the colour range endpoints.'
        ),

        # ── Node ─────────────────────────────────────────────────────────────
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

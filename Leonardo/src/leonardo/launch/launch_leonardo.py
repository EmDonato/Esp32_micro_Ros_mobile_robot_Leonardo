#!/usr/bin/env python3
"""
@file teleop_launch.py
@brief Launch file for joystick teleoperation and robot control nodes.

This launch description initializes:
  - joy_node for game controller input
  - teleop_twist_joy node for converting joystick input to Twist commands
  - control_teleop node for wheel speed computation
  - static and odom transform broadcasters
  - micro-ROS agent for embedded communication via UDP6
"""
import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    Generate and return the ROS2 launch description.

    @returns LaunchDescription containing all configured actions.
    """
    ld = LaunchDescription()

    # -----------------------------------------------------------------------
    # Joystick input: publishes sensor_msgs/Joy messages
    # Parameters:
    #   device_id             ID of joystick device (/dev/input/jsX)
    #   deadzone              Minimum axis movement to register
    #   autorepeat_rate       Rate at which buttons repeat when held
    #   sticky_buttons        If True, buttons latch until read
    #   coalesce_interval_ms  Time window to combine events
    ld.add_action(Node(
        package='joy',
        executable='joy_node',
        name='game_controller',
        parameters=[{
            'device_id': 1,
            'deadzone': 0.05,
            'autorepeat_rate': 0.0,
            'sticky_buttons': False,
            'coalesce_interval_ms': 1
        }],
        output='screen'
    ))

    # -----------------------------------------------------------------------
    # Teleoperation: map joystick axes/buttons to geometry_msgs/Twist
    # Parameters:
    #   require_enable_button  Must hold enable_button to send commands
    #   enable_button          Button index to enable teleop
    #   enable_turbo_button    Button index for turbo mode (-1=disabled)
    #   axis_linear.*, axis_angular.*  Axis indices for motion
    #   scale_linear.*, scale_angular.* Scaling factors for speed
    #   publish_stamped_twist  If True, publish TwistStamped
    ld.add_action(Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy',
        parameters=[{
            'require_enable_button': True,
            'enable_button': 5,
            'enable_turbo_button': -1,
            'axis_linear.x': 1,
            'axis_linear.y': 2,
            'axis_linear.z': 3,
            'scale_linear.x': 0.8,
            'scale_linear.y': 1.0,
            'scale_linear.z': 0.0,
            'axis_angular.yaw': 2,
            'scale_angular.yaw': 0.5,
            'publish_stamped_twist': False
        }],
        remappings=[
        	('cmd_vel', '/cmd_vel_joy')
        ],
        output='screen'
    ))
    # -----------------------------------------------------------------------    
    ld.add_action(Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        parameters=[{
            '/home/genesis/Desktop/Leonardo/config/config_param_cmd_vel_mux.yaml'
        }],
        remappings=[
        	('cmd_vel_out', '/cmd_vel_mux')
        ],
        output='screen'
    ))
    # -----------------------------------------------------------------------
    # Control Teleop: custom node converting Twist to wheel commands
    ld.add_action(Node(
        package='leonardo',
        executable='control_teleop',
        name='control_teleop',
        output='screen'
    ))

    # -----------------------------------------------------------------------
    # Static transforms: broadcast frames for odometry and laser
    ld.add_action(Node(
        package='odom_to_tf_cpp',
        executable='odom_to_tf_node',
        name='odom_to_tf',
        output='screen'
    ))
    ld.add_action(Node(
        package='laser_tf_cpp',
        executable='static_tf_base_laser',
        name='static_tf_base_laser',
        output='screen'
    ))

    # -----------------------------------------------------------------------
    # Micro-ROS agent: bridge for microcontroller communication via UDP6
    ld.add_action(ExecuteProcess(
        cmd=[
            'bash', '-c',
            'source ~/Desktop/agent/install/local_setup.bash && '
            'ros2 run micro_ros_agent micro_ros_agent udp6 --port 8888'
        ],
        output='screen'
    ))
    urdf_file = os.path.join(
        get_package_share_directory('leonardo'),
        'urdf',
        'unicycle.urdf'
    )
    # -----------------------------------------------------------------------
    # Time Synchronization Node: publishes current ROS time periodically
    ld.add_action(Node(
        package='time_sync_publisher',
        executable='time_sync_publisher',
        output='screen'
    ))
    
    # -----------------------------------------------------------------------
    # cmd_vel from nav2 normalize
    ld.add_action(Node(
        package='cmd_vel_normalizer',
        executable='cmd_vel_normalizer_node',
        output='screen'
    ))    
    
    ld.add_action(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        arguments=[urdf_file],
        output='screen'
    ))
    return ld


#!/usr/bin/env python3
"""
Launch file for EMOROBCARE LED Service

This launch file starts the LED service node with configurable parameters.

Usage:
    ros2 launch emorobcare_led_service led_service.launch.py
    
    # With custom parameters:
    ros2 launch emorobcare_led_service led_service.launch.py i2c_bus:=1 i2c_addr:=8 num_leds:=5
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for LED service."""
    
    # Declare launch arguments
    i2c_bus_arg = DeclareLaunchArgument(
        'i2c_bus',
        default_value='1',
        description='I2C bus number (default: 1)'
    )
    
    i2c_addr_arg = DeclareLaunchArgument(
        'i2c_addr',
        default_value='8',
        description='I2C address in decimal (default: 8 = 0x08)'
    )
    
    num_leds_arg = DeclareLaunchArgument(
        'num_leds',
        default_value='5',
        description='Number of LEDs in the strip (default: 5)'
    )
    
    # LED service node
    led_service_node = Node(
        package='emorobcare_led_service',
        executable='led_service_node',
        name='led_service_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'i2c_bus': LaunchConfiguration('i2c_bus'),
            'i2c_addr': LaunchConfiguration('i2c_addr'),
            'num_leds': LaunchConfiguration('num_leds'),
        }],
        respawn=True,
        respawn_delay=2.0,
    )
    
    # Log info
    log_info = LogInfo(
        msg='Starting EMOROBCARE LED Service...'
    )
    
    return LaunchDescription([
        log_info,
        i2c_bus_arg,
        i2c_addr_arg,
        num_leds_arg,
        led_service_node,
    ])

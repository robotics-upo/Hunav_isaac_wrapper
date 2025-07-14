#!/usr/bin/env python3
"""
Launch file for HuNav Isaac Wrapper simulation.
Launches the main script using the ROS2 launcher, which handles Isaac Sim python detection.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description that uses the ROS2 launcher."""
    
    # Declare launch arguments
    scenario_arg = DeclareLaunchArgument(
        'scenario',
        default_value='',
        description='Scenario configuration file to use (optional - will use interactive mode if not specified)'
    )
    
    batch_arg = DeclareLaunchArgument(
        'batch',
        default_value='false',
        description='Run in batch mode (non-interactive)'
    )
    
    extra_args_arg = DeclareLaunchArgument(
        'extra_args',
        default_value='',
        description='Additional arguments to pass to the main script'
    )
    
    # Launch configurations
    scenario = LaunchConfiguration('scenario')
    batch = LaunchConfiguration('batch')
    extra_args = LaunchConfiguration('extra_args')
    
    # Use the ROS2 launcher which handles Isaac Sim python detection
    launcher_with_scenario = ExecuteProcess(
        cmd=['ros2', 'run', 'hunav_isaac_wrapper', 'hunav_isaac_launcher',
             '--config', scenario, '--batch', extra_args],
        condition=IfCondition(scenario),
        output='screen',
        name='hunav_isaac_launcher_scenario'
    )
    
    # Interactive mode (default when no scenario specified)
    launcher_interactive = ExecuteProcess(
        cmd=['ros2', 'run', 'hunav_isaac_wrapper', 'hunav_isaac_launcher', extra_args],
        condition=UnlessCondition(scenario),
        output='screen',
        name='hunav_isaac_launcher_interactive'
    )
    
    return LaunchDescription([
        # Launch arguments
        scenario_arg,
        batch_arg,
        extra_args_arg,
        
        # Log launch info
        LogInfo(msg=['Launching HuNav Isaac Wrapper...']),
        LogInfo(msg=['Use scenario parameter to specify a scenario file']),
        LogInfo(msg=['Otherwise interactive mode will start']),
        
        # Launcher processes
        launcher_with_scenario,
        launcher_interactive,
    ])

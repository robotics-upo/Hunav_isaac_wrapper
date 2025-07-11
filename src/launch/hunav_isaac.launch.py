#!/usr/bin/env python3
"""
Simple launch file for HuNav Isaac Wrapper simulation.
Launches the main script directly, similar to: bash ~/isaacsim/python.sh ~/Hunav_isaac_wrapper/main.py
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, ExecuteProcess
from launch.substitutions import LaunchConfiguration, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate a simple launch description that mimics the original bash command."""
    
    # Package directory
    pkg_dir = FindPackageShare('hunav_isaac_wrapper')
    
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
    
    # Launch configurations
    scenario = LaunchConfiguration('scenario')
    batch = LaunchConfiguration('batch')
    
    # Build the command arguments
    main_script = PathJoinSubstitution([pkg_dir, '..', 'scripts', 'main.py'])
    
    # Execute the main script directly (similar to original usage)
    main_process = ExecuteProcess(
        cmd=[
            'python3', main_script,
            '--scenario', scenario,
            '--batch'
        ],
        condition=lambda context: context.launch_configurations['batch'] == 'true' and context.launch_configurations['scenario'] != '',
        output='screen',
        name='hunav_isaac_main_batch'
    )
    
    # Interactive mode (default)
    interactive_process = ExecuteProcess(
        cmd=['python3', main_script],
        condition=lambda context: context.launch_configurations['batch'] == 'false' or context.launch_configurations['scenario'] == '',
        output='screen',
        name='hunav_isaac_main_interactive'
    )
    
    return LaunchDescription([
        # Launch arguments
        scenario_arg,
        batch_arg,
        
        # Log launch info
        LogInfo(msg=['Launching HuNav Isaac Wrapper...']),
        LogInfo(msg=['Use scenario parameter to specify a scenario file, otherwise interactive mode will start']),
        
        # Processes
        main_process,
        interactive_process,
    ])

# import numpy as np
# import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals, IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ========================================
    # Declare launch parameters
    # ========================================
    declared_arguments = []

    # ========================================
    # Launch nodes
    # ========================================
    nodes = []

    # Launch controller manager
    # ---------------------------------------
    nodes += Node(
        package='vic_ppf_python',
        executable='bednarczyk_node',
        name='SIPF_W2_node',
        remappings=[],
        parameters=[
            global_setting,
            {
                'passivation_function': 'bednarczyk_W2'
            },
        ],
    ),

    # Load controllers
    # ---------------------------------------
    
    # TODO

    # Launch dummy reference
    # ---------------------------------------
    
    # TODO

    # ========================================
    # Return launch description
    # ========================================
    return LaunchDescription(
        declared_arguments
        + nodes
    )

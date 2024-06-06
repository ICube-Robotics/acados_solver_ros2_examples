# Copyright 2024 ICUBE Laboratory, University of Strasbourg
# License: Apache License, Version 2.0

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument  # noqa: F401
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ========================================
    # Declare launch parameters
    # ========================================
    declared_arguments = []

    # ========================================
    # Get config files
    # ========================================
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("example_acados_description"),
                    "urdf",
                    "rrbot.urdf.xacro",
                ]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    config_controllers = PathJoinSubstitution(
        [
            FindPackageShare("example_acados_bringup"),
            "config",
            "controllers.yaml",
        ]
    )
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("example_acados_description"), "rviz", "rrbot.rviz"]
    )

    # ========================================
    # Launch nodes
    # ========================================

    # Launch nodes
    # ---------------------------------------
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, config_controllers],
        output="both",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    interactive_cartesian_reference_node = Node(
        package="example_acados_bringup",
        executable="interactive_cartesian_reference",
        output="both",
    )

    # Load controllers
    # ---------------------------------------

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager"
        ],
    )

    nmpc_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "nmpc_controller",
            "--controller-manager", "/controller_manager"
        ],
    )

    # Launch dummy reference
    # ---------------------------------------

    # TODO

    # ========================================
    # Return launch description
    # ========================================
    nodes = [
        control_node,
        robot_state_pub_node,
        rviz_node,
        interactive_cartesian_reference_node,
        joint_state_broadcaster_spawner,
        nmpc_controller_spawner,
    ]

    return LaunchDescription(
        declared_arguments
        + nodes
    )

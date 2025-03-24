"""
@file visual_control.launch.py
@brief Launch file for setting up the robot with visual servoing components, without joint_state_broadcaster.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    ExecuteProcess,
    RegisterEventHandler,
    DeclareLaunchArgument,
    TimerAction,
)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
import xacro
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # Declare the 'num_cameras' launch argument
    num_cameras_arg = DeclareLaunchArgument(
        'num_cameras',
        default_value='1',
        description='Number of cameras (1 or 2)'
    )
    num_cameras = LaunchConfiguration('num_cameras')

    # Package Directories
    pkg_name = "robot_description_tb"
    robot_moveit_config = "robot_moveit_config_tb"
    share_dir = get_package_share_directory(pkg_name)
    moveit_config_pkg_path = get_package_share_directory(robot_moveit_config)

    # Load and process URDF/XACRO file
    xacro_file = os.path.join(share_dir, "urdf", "r5a_v_ros.urdf.xacro")
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {"robot_description": robot_description_config.toxml()}

    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": False}],
    )

    # Aruco Detector Single Node
    aruco_detector_single_node = Node(
        package="robot_visual",
        executable="aruco_detector_single",
        output="screen",
        parameters=[{"use_sim_time": False}],
        condition=UnlessCondition(PythonExpression(['"', num_cameras, '" == "2"']))
    )

    # Aruco Detector Double Node (only if num_cameras == 2)
    aruco_detector_double_node = Node(
        package="robot_visual",
        executable="aruco_detector_double",
        output="screen",
        parameters=[{"use_sim_time": False}],
        condition=IfCondition(PythonExpression(['"', num_cameras, '" == "2"']))
    )

    # Visual Joint State Publisher Node
    visual_joint_state_publisher_node = Node(
        package="robot_control_tb",
        executable="visual_joint_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": False}],
    )

    # Joint State Bridge Node
    joint_state_bridge_node = Node(
        package="robot_control_tb",
        executable="joint_state_bridge",
        output="screen",
        parameters=[{"use_sim_time": False}],
    )

    # Controller Manager Node
    controller_config = os.path.join(share_dir, "config", "controllers.yaml")
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[robot_description, controller_config],
    )

    # Load Controllers
    load_arm_controller = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active", "arm_controller"],
        output="screen",
    )

    load_gripper_controller = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active", "gripper_controller"],
        output="screen",
    )

    # MoveIt Configuration
    moveit_config = (
        MoveItConfigsBuilder(robot_moveit_config, package_name=robot_moveit_config)
        .robot_description(file_path=xacro_file, mappings={"use_sim_time": "false"})
        .robot_description_semantic(os.path.join(moveit_config_pkg_path, "config", "armr5.srdf"))
        .robot_description_kinematics(os.path.join(moveit_config_pkg_path, "config", "kinematics.yaml"))
        .trajectory_execution(os.path.join(moveit_config_pkg_path, "config", "moveit_controllers.yaml"))
        .planning_scene_monitor(publish_robot_description=True, publish_robot_description_semantic=True)
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # Move Group Node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), {"use_sim_time": False}],
    )

    # GUI Node
    gui_node = Node(
        package="robot_control_tb",
        executable="moveit_control_gui",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": False},
            {"moveit_current_state_monitor.joint_state_qos": "sensor_data"},
        ],
    )

    # Event Handlers to Enforce Order
    aruco_start_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=robot_state_publisher_node,
            on_start=[aruco_detector_single_node, aruco_detector_double_node]
        )
    )

    visual_joint_start_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=aruco_detector_single_node,
            on_start=[visual_joint_state_publisher_node]
        )
    )

    joint_state_bridge_start_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=visual_joint_state_publisher_node,
            on_start=[joint_state_bridge_node]
        )
    )

    controller_manager_start_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=joint_state_bridge_node,
            on_start=[controller_manager_node]
        )
    )

    arm_controller_start_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager_node,
            on_start=[load_arm_controller]
        )
    )

    gripper_controller_start_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_arm_controller,
            on_exit=[load_gripper_controller]
        )
    )

    move_group_start_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_gripper_controller,
            on_exit=[move_group_node]
        )
    )

    gui_start_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=move_group_node,
            on_start=[gui_node]
        )
    )

    # Launch Description
    return LaunchDescription([
        num_cameras_arg,
        robot_state_publisher_node,
        aruco_start_handler,
        visual_joint_start_handler,
        joint_state_bridge_start_handler,
        move_group_start_handler,
        controller_manager_start_handler,
        arm_controller_start_handler,
        gripper_controller_start_handler,
        gui_start_handler,

    ])

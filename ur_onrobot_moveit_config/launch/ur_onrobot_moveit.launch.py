# -*- coding: utf-8 -*-
# SPDX-License-Identifier: Apache-2.0
# --------------------------------------------------------------
#  MoveIt + UR + OnRobot (2FG7 / RG2 / RG6) – works with real robot or URSim
# --------------------------------------------------------------

import os
import socket
import subprocess
from typing import List

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    SetLaunchConfiguration,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue, ParameterFile
from launch_ros.substitutions import FindPackageShare

# ----------------------------------------------------------------------
# Global state (only used inside OpaqueFunctions)
# ----------------------------------------------------------------------
chosen_robot_ip: str | None = None
camera_enabled: bool = False


def ping_ip(ip: str, timeout: float = 1.0);x -> bool:
    try:
        with socket.create_connection((ip, 30003), timeout=timeout):
            return True
    except Exception:
        return False


def detect_robot(context) -> List[SetLaunchConfiguration]:
    global chosen_robot_ip
    candidates = [
        ip.strip()
        for ip in LaunchConfiguration("robot_ip_candidates")
        .perform(context)
        .split(",")
        if ip.strip()
    ]
    timeout = float(LaunchConfiguration("connection_timeout").perform(context))

    print(f"[INFO] Probing robot IPs: {candidates} (timeout={timeout}s)")
    for ip in candidates:
        if ping_ip(ip, timeout):
            chosen_robot_ip = ip
            print(f"[INFO] Robot FOUND at {ip} → REAL hardware")
            return [SetLaunchConfiguration("detected_mode", "false")]

    # fallback → URSim
    chosen_robot_ip = "127.0.0.1"
    print("[INFO] No robot → URSim (fake mode)")
    return [SetLaunchConfiguration("detected_mode", "true")]


def check_camera(context) -> List:
    global camera_enabled
    use_camera = LaunchConfiguration("use_camera").perform(context)

    if use_camera == "true":
        camera_enabled = True
        print("[INFO] use_camera:=true → OctoMap ON")
    elif use_camera == "false":
        camera_enabled = False
        print("[INFO] use_camera:=false → OctoMap OFF")
    else:  # auto
        try:
            out = subprocess.run(
                ["ros2", "topic", "list"], capture_output=True, text=True, timeout=2
            )
            camera_enabled = "/camera/depth/image_rect_raw" in out.stdout
        except Exception:
            camera_enabled = False
        print(
            "[INFO] "
            + ("RealSense detected → OctoMap ON" if camera_enabled else "No RealSense → OctoMap OFF")
        )
    return []


# ----------------------------------------------------------------------
def launch_setup(context, *_, **__) -> List:
    global chosen_robot_ip, camera_enabled

    # ------------------- Arguments -------------------
    ur_type = LaunchConfiguration("ur_type")
    onrobot_type = LaunchConfiguration("onrobot_type")
    tf_prefix = LaunchConfiguration("tf_prefix")
    launch_rviz = LaunchConfiguration("launch_rviz")
    launch_servo = LaunchConfiguration("launch_servo")
    use_sim_time = LaunchConfiguration("use_sim_time")
    detected_mode = LaunchConfiguration("detected_mode")   # "true" = fake

    # ------------------- URDF (xacro) -------------------
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("ur_onrobot_description"), "urdf", "ur_onrobot.urdf.xacro"]
            ),
            f" robot_ip:={chosen_robot_ip}",
            f" ur_type:={ur_type}",
            f" onrobot_type:={onrobot_type}",
            f" tf_prefix:={tf_prefix}",
            f" use_fake_hardware:={detected_mode}",
            " connection_type:=tcp",
            f" ip_address:={chosen_robot_ip}",
            " port:=502",
            " device_address:=65",
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # ------------------- SRDF (xacro) -------------------
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("ur_onrobot_moveit_config"), "srdf", "ur_onrobot.srdf.xacro"]
            ),
            " name:=ur_onrobot",
            f" prefix:={tf_prefix}",
            f" onrobot_type:={onrobot_type}",
        ]
    )
    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(
            robot_description_semantic_content, value_type=str
        )
    }

    # ------------------- Kinematics -------------------
    kinematics_yaml = {
        "robot_description_kinematics": {
            "ur_onrobot_manipulator": {
                "kinematics_solver": "kdl_kinematics_plugin/KDLKinematicsPlugin",
                "kinematics_solver_search_resolution": 0.005,
                "kinematics_solver_timeout": 0.05,
            }
        }
    }

    # ------------------- Controllers (MUST be ParameterFile) -------------------
    controllers_file = ParameterFile(
        PathJoinSubstitution(
            [FindPackageShare("ur_onrobot_moveit_config"), "config", "controllers.yaml"]
        ),
        allow_substs=True,
    )

    # ------------------- ros2_control node -------------------
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="controller_manager",
        output="screen",
        parameters=[
            robot_description,
            controllers_file,
            {"use_fake_hardware": PythonExpression(["'", detected_mode, "' == 'true'"])},
            {"robot_ip": chosen_robot_ip},
            {"reverse_port": 50001},
            {"tf_prefix": tf_prefix},
        ],
    )

    # ------------------- Spawners (real = scaled, fake = un-scaled) -------------------
    def spawner(name: str):
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[name, "--controller-manager", "/controller_manager"],
            output="screen",
        )

    spawner_real = TimerAction(
        period=5.0,
        actions=[
            spawner("scaled_joint_trajectory_controller"),
            spawner("finger_width_trajectory_controller"),
        ],
        condition=UnlessCondition(detected_mode),
    )
    spawner_fake = TimerAction(
        period=5.0,
        actions=[
            spawner("joint_trajectory_controller"),
            spawner("finger_width_trajectory_controller"),
        ],
        condition=IfCondition(detected_mode),
    )

    # ------------------- MoveGroup -------------------
    move_group_params = [
        robot_description,
        robot_description_semantic,
        kinematics_yaml,
        {"use_sim_time": use_sim_time},
        # ---- tell MoveIt to use ros2_control ----
        {"moveit_controller_manager": "ros2_control"},
        {"moveit_controller_manager_name": "controller_manager"},
    ]

    if camera_enabled:
        sensors_yaml = {
            "sensors": [
                {
                    "sensor_plugin": "occupancy_map_monitor/PointCloudOctomapUpdater",
                    "point_cloud_topic": "/camera/depth/color/points",
                    "max_range": 5.0,
                    "padding_offset": 0.01,
                    "padding_scale": 1.0,
                    "point_subsample": 1,
                    "filtered_cloud_topic": "filtered_cloud",
                }
            ]
        }
        move_group_params.append({"sensors_3d": sensors_yaml})

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=move_group_params,
    )

    # ------------------- RViz -------------------
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("ur_onrobot_moveit_config"), "rviz", "view_robot.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            {"use_sim_time": use_sim_time},
        ],
        condition=IfCondition(launch_rviz),
    )

    # ------------------- Servo (optional) -------------------
    servo_yaml = {
        "moveit_servo": {
            "use_gazebo": False,
            "command_in_type": "unitless",
            "publish_period": 0.02,
            "collision_check_rate": 50,
            "incoming_command_timeout": 0.1,
        }
    }
    servo_container = Node(
        package="rclcpp_components",
        executable="component_container",
        name="servo_container",
        output="screen",
        parameters=[
            servo_yaml,
            robot_description,
            robot_description_semantic,
            {"moveit_servo.move_group_name": "ur_onrobot_manipulator"},
        ],
        composable_node_descriptions=[
            {
                "type": "moveit_servo::ServoNode",
                "name": "servo_node",
                "namespace": "",
                "parameters": [],
            }
        ],
        condition=IfCondition(launch_servo),
    )

    # ------------------- Assemble -------------------
    delayed_nodes = TimerAction(
        period=8.0,
        actions=[move_group_node, rviz_node, servo_container],
    )

    return [
        control_node,
        spawner_real,
        spawner_fake,
        delayed_nodes,
    ]


# ----------------------------------------------------------------------
def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            # ------------------- Arguments -------------------
            DeclareLaunchArgument("ur_type", default_value="ur5e", description="UR series"),
            DeclareLaunchArgument(
                "onrobot_type",
                default_value="2fg7",
                description="OnRobot gripper",
                choices=["rg2", "rg6", "2fg7", "2fg14"],
            ),
            DeclareLaunchArgument("tf_prefix", default_value="", description="tf prefix"),
            DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?"),
            DeclareLaunchArgument("launch_servo", default_value="true", description="Launch Servo?"),
            DeclareLaunchArgument("use_sim_time", default_value="false", description="Use sim time"),
            DeclareLaunchArgument(
                "use_camera",
                default_value="auto",
                description="'auto' (detect), 'true' (force on), 'false' (force off)",
            ),
            DeclareLaunchArgument(
                "robot_ip_candidates",
                default_value="192.168.1.101,192.168.1.105",
                description="Comma-separated IPs",
            ),
            DeclareLaunchArgument(
                "connection_timeout", default_value="1.5", description="TCP timeout per IP"
            ),
            DeclareLaunchArgument(
                "detected_mode",
                default_value="true",  # fallback = fake
                description="Internal: true = fake",
            ),
            # ------------------- Opaque functions -------------------
            OpaqueFunction(function=detect_robot),
            OpaqueFunction(function=check_camera),
            OpaqueFunction(function=launch_setup),
        ]
    )
# Copyright (c) 2021 PickNik, Inc.
# SPDX-License-Identifier: Apache-2.0

import os
import socket
import subprocess
from typing import List

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
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
from launch_ros.actions import Node, ComposableNodeContainer, SetLaunchConfiguration
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

from ur_onrobot_moveit_config.launch_common import load_yaml


# ----------------------------------------------------------------------
# Global state – only used inside the OpaqueFunctions
# ----------------------------------------------------------------------
camera_detected: bool = False
camera_enabled: bool = False
chosen_robot_ip: str | None = None


def ping_ip(ip: str, timeout: float = 1.0) -> bool:
    """Ping the UR primary interface (port 30003)."""
    try:
        with socket.create_connection((ip, 30003), timeout=timeout):
            return True
    except Exception:
        return False


def detect_robot(context) -> List[SetLaunchConfiguration]:
    """Try every candidate IP → set chosen_robot_ip + detected_mode."""
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

    # ----- fallback to URSim -----
    chosen_robot_ip = "127.0.0.1"
    print("[INFO] No robot responded → URSim (fake mode)")
    return [SetLaunchConfiguration("detected_mode", "true")]


def check_camera_and_decide(context) -> List:
    """Detect RealSense → set camera_enabled (used later by MoveGroup)."""
    global camera_enabled, camera_detected
    use_camera = LaunchConfiguration("use_camera").perform(context)

    # ---- detect topic ----
    try:
        out = subprocess.run(
            ["ros2", "topic", "list"], capture_output=True, text=True, timeout=2
        )
        camera_detected = "/camera/depth/image_rect_raw" in out.stdout
    except Exception:
        camera_detected = False

    if use_camera == "true":
        camera_enabled = True
        print("[INFO] use_camera:=true → OctoMap forced ON")
    elif use_camera == "false":
        camera_enabled = False
        print("[INFO] use_camera:=false → OctoMap disabled")
    else:  # auto
        camera_enabled = camera_detected
        print(
            "[INFO] "
            + ("RealSense detected → OctoMap ON" if camera_detected else "No RealSense → OctoMap OFF")
        )
    return []  # nothing to launch here


# ----------------------------------------------------------------------
def launch_setup(context, *_, **__) -> List:
    global camera_enabled, chosen_robot_ip

    # ------------------------------------------------------------------
    # Launch arguments
    # ------------------------------------------------------------------
    ur_type = LaunchConfiguration("ur_type")
    onrobot_type = LaunchConfiguration("onrobot_type")
    prefix = LaunchConfiguration("prefix")
    launch_rviz = LaunchConfiguration("launch_rviz")
    launch_servo = LaunchConfiguration("launch_servo")
    use_sim_time = LaunchConfiguration("use_sim_time")
    moveit_pkg = LaunchConfiguration("moveit_config_package")
    moveit_srdf = LaunchConfiguration("moveit_config_file")
    warehouse_path = LaunchConfiguration("warehouse_sqlite_path")
    detected_mode = LaunchConfiguration("detected_mode")  # "true" or "false"

    # ------------------------------------------------------------------
    # URDF (xacro) – works for 2fg7, rg2, rg6 automatically
    # ------------------------------------------------------------------
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("ur_onrobot_description"), "urdf", "ur_onrobot.urdf.xacro"]
            ),
            " robot_ip:=", chosen_robot_ip,
            " ur_type:=", ur_type,
            " onrobot_type:=", onrobot_type,
            " prefix:=", prefix,
            " use_fake_hardware:=", detected_mode,
            " connection_type:=tcp",
            " ip_address:=", chosen_robot_ip,          # <-- dynamic gripper IP
            " port:=502",
            " device_address:=65",
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # ------------------------------------------------------------------
    # SRDF (xacro)
    # ------------------------------------------------------------------
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(moveit_pkg), "srdf", moveit_srdf]),
            " name:=ur_onrobot",
            " prefix:=", prefix,
            " onrobot_type:=", onrobot_type,
        ]
    )
    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(
            robot_description_semantic_content, value_type=str
        )
    }

    # ------------------------------------------------------------------
    # Kinematics
    # ------------------------------------------------------------------
    kinematics_yaml = load_yaml("ur_onrobot_moveit_config", "config/kinematics.yaml")
    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}

    # ------------------------------------------------------------------
    # OMPL planning pipeline
    # ------------------------------------------------------------------
    ompl_planning_pipeline = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": " ".join(
                [
                    "default_planner_request_adapters/AddTimeOptimalParameterization",
                    "default_planner_request_adapters/FixWorkspaceBounds",
                    "default_planner_request_adapters/FixStartStateBounds",
                    "default_planner_request_adapters/FixStartStateCollision",
                    "default_planner_request_adapters/FixStartStatePathConstraints",
                ]
            ),
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_yaml = load_yaml("ur_onrobot_moveit_config", "config/ompl_planning.yaml")
    ompl_planning_pipeline["move_group"].update(ompl_planning_yaml)

    # ------------------------------------------------------------------
    # Controllers (the file you already have)
    # ------------------------------------------------------------------
    controllers_file = PathJoinSubstitution(
        [FindPackageShare("ur_onrobot_moveit_config"), "config", "controllers.yaml"]
    )

    # ------------------------------------------------------------------
    # ros2_control node
    # ------------------------------------------------------------------
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="controller_manager",
        output="screen",
        parameters=[robot_description, controllers_file],
    )

    # ------------------------------------------------------------------
    # UR driver (real hardware)
    # ------------------------------------------------------------------
    ur_driver = Node(
        package="ur_robot_driver",
        executable="ur_ros2_control_node",
        name="ur_control_node",
        output="screen",
        parameters=[
            robot_description,
            {"robot_ip": chosen_robot_ip},
            {"use_fake_hardware": PythonExpression(["'", detected_mode, "' == 'false'"])},
            {"reverse_port": 50001},
            {"tf_prefix": prefix},
        ],
        condition=UnlessCondition(detected_mode),  # real mode
    )

    # ------------------------------------------------------------------
    # URSim Docker (fake mode)
    # ------------------------------------------------------------------
    ursim_docker = ExecuteProcess(
        cmd=[
            "docker",
            "run",
            "-d",
            "--rm",
            "--name",
            "ursim_auto",
            "-p",
            "59000:59000",
            "-p",
            "29999:29999",
            "-p",
            "30001-30004:30001-30004",
            "universalrobots/ursim_e-series",
            "ur_driver",
        ],
        output="screen",
        condition=IfCondition(detected_mode),
    )

    # ------------------------------------------------------------------
    # Spawners – scaled (real) / un-scaled (fake) + gripper
    # ------------------------------------------------------------------
    def spawner_node(name: str):
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[name, "--controller-manager", "/controller_manager"],
            output="screen",
        )

    spawner_scaled = TimerAction(
        period=5.0,
        actions=[
            spawner_node("scaled_joint_trajectory_controller"),
            spawner_node("finger_width_trajectory_controller"),
        ],
        condition=UnlessCondition(detected_mode),
    )

    spawner_unscaled = TimerAction(
        period=5.0,
        actions=[
            spawner_node("joint_trajectory_controller"),
            spawner_node("finger_width_trajectory_controller"),
        ],
        condition=IfCondition(detected_mode),
    )

    # ------------------------------------------------------------------
    # MoveGroup – adds OctoMap only when camera_enabled == True
    # ------------------------------------------------------------------
    move_group_params = [
        robot_description,
        robot_description_semantic,
        robot_description_kinematics,
        ompl_planning_pipeline,
        {"use_sim_time": use_sim_time},
        {"planning_scene_monitor_options.publish_planning_scene": True},
    ]

    if camera_enabled:
        sensors_yaml = load_yaml("ur_onrobot_moveit_config", "config/sensors_3d.yaml")
        move_group_params.append({"octomap_updater": sensors_yaml})

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=move_group_params,
    )

    # ------------------------------------------------------------------
    # RViz
    # ------------------------------------------------------------------
    rviz_config = PathJoinSubstitution(
        [FindPackageShare(moveit_pkg), "rviz", "view_robot.rviz"]
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
            {"use_sim_time": use_sim_time},
        ],
        condition=IfCondition(launch_rviz),
    )

    # ------------------------------------------------------------------
    # Servo
    # ------------------------------------------------------------------
    servo_yaml = load_yaml("ur_onrobot_moveit_config", "config/ur_onrobot_servo.yaml")
    servo_container = ComposableNodeContainer(
        name="servo_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="moveit_servo",
                plugin="moveit_servo::ServoNode",
                name="servo_node",
                parameters=[
                    servo_yaml,
                    robot_description,
                    robot_description_semantic,
                    {"moveit_servo.move_group_name": "ur_onrobot_manipulator"},
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            )
        ],
        output="screen",
        condition=IfCondition(launch_servo),
    )

    # ------------------------------------------------------------------
    # Delayed start
    # ------------------------------------------------------------------
    delayed_nodes = TimerAction(
        period=8.0,
        actions=[move_group_node, rviz_node, servo_container],
    )

    # ------------------------------------------------------------------
    # Assemble launch description
    # ------------------------------------------------------------------
    actions = [
        control_node,
        ur_driver,
        ursim_docker,
        spawner_scaled,
        spawner_unscaled,
        delayed_nodes,
    ]
    return [a for a in actions if a is not None]


# ----------------------------------------------------------------------
def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            # ------------------- Launch arguments -------------------
            DeclareLaunchArgument(
                "ur_type",
                default_value="ur5e",
                description="UR series",
                choices=[
                    "ur3",
                    "ur3e",
                    "ur5",
                    "ur5e",
                    "ur10",
                    "ur10e",
                    "ur16e",
                    "ur20",
                    "ur30",
                ],
            ),
            DeclareLaunchArgument(
                "onrobot_type",
                default_value="2fg7",
                description="OnRobot gripper type",
                choices=["rg2", "rg6", "2fg7", "2fg14"],
            ),
            DeclareLaunchArgument("prefix", default_value='""', description="Prefix for joint names"),
            DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?"),
            DeclareLaunchArgument("launch_servo", default_value="true", description="Launch Servo?"),
            DeclareLaunchArgument("use_sim_time", default_value="false", description="Use sim time"),
            DeclareLaunchArgument(
                "use_camera",
                default_value="auto",
                description="'auto' (detect), 'true' (force on), 'false' (force off)",
            ),
            DeclareLaunchArgument(
                "moveit_config_package", default_value="ur_onrobot_moveit_config"
            ),
            DeclareLaunchArgument(
                "moveit_config_file", default_value="ur_onrobot.srdf.xacro"
            ),
            DeclareLaunchArgument(
                "warehouse_sqlite_path",
                default_value=os.path.expanduser("~/.ros/warehouse_ros.sqlite"),
            ),
            # ------------------- Auto-detection -------------------
            DeclareLaunchArgument(
                "robot_ip_candidates",
                default_value="192.168.1.101,192.168.1.105",
                description="Comma-separated list of robot IPs to try",
            ),
            DeclareLaunchArgument(
                "connection_timeout", default_value="1.5", description="TCP timeout per IP (seconds)"
            ),
            DeclareLaunchArgument(
                "detected_mode",
                default_value="true",  # fallback = fake
                description="Internal flag: 'true' = fake, 'false' = real",
            ),
            # ------------------- Opaque functions -------------------
            OpaqueFunction(function=detect_robot),
            OpaqueFunction(function=check_camera_and_decide),
            OpaqueFunction(function=launch_setup),
        ]
    )
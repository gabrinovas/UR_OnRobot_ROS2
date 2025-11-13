# Copyright (c) 2021 PickNik, Inc.  (license unchanged)

import os
import socket
import subprocess
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    TimerAction,
    ExecuteProcess,
    GroupAction,
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    FindExecutable,
    Command,
    PythonExpression,
)
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ur_onrobot_moveit_config.launch_common import load_yaml


# ----------------------------------------------------------------------
# Global state
camera_detected = False
camera_enabled = False
chosen_robot_ip = None  # will be set by detect_robot()


def ping_ip(ip, timeout=1.0):
    """Return True if host responds to a TCP ping on port 30003 (UR primary interface)."""
    try:
        with socket.create_connection((ip, 30003), timeout=timeout):
            return True
    except (OSError, socket.timeout):
        return False


def detect_robot(context, *args, **kwargs):
    """Try the candidate IPs → decide real/fake and set globals + launch config."""
    global chosen_robot_ip

    candidates_str = LaunchConfiguration("robot_ip_candidates").perform(context)
    timeout = float(LaunchConfiguration("connection_timeout").perform(context))

    # Parse candidates
    candidates = [ip.strip() for ip in candidates_str.split(",") if ip.strip()]

    print(f"[INFO] Checking robot IPs: {candidates} (timeout={timeout}s)")

    for ip in candidates:
        if ping_ip(ip, timeout):
            chosen_robot_ip = ip
            print(f"[INFO] Robot FOUND at {ip} → using REAL hardware")
            # Set launch config for real mode
            yield SetLaunchConfiguration("detected_mode", "false")
            return

    # No robot found
    chosen_robot_ip = "127.0.0.1"  # irrelevant for fake mode
    print("[INFO] No robot responded → falling back to URSim (fake mode)")
    # Set launch config for fake mode
    yield SetLaunchConfiguration("detected_mode", "true")


def check_camera_and_decide(context):
    global camera_detected, camera_enabled
    use_camera_val = LaunchConfiguration("use_camera").perform(context)

    try:
        result = subprocess.run(
            ["ros2", "topic", "list"], capture_output=True, text=True, timeout=2
        )
        topics = result.stdout.splitlines()
        camera_detected = "/camera/depth/image_rect_raw" in topics
    except Exception:
        camera_detected = False

    if use_camera_val == "true":
        camera_enabled = True
        print("[INFO] use_camera:=true → Forcing OctoMap ON")
    elif use_camera_val == "false":
        camera_enabled = False
        print("[INFO] use_camera:=false → OctoMap disabled")
    else:
        camera_enabled = camera_detected
        print(
            f"[INFO] {'RealSense detected → OctoMap ON' if camera_detected else 'No RealSense → OctoMap OFF'}"
        )
    return []


# ----------------------------------------------------------------------
def launch_setup(context, *args, **kwargs):
    global camera_enabled, chosen_robot_ip

    detected_mode = LaunchConfiguration("detected_mode")
    use_fake_str = PythonExpression(["'", detected_mode, "'"])  # 'true' or 'false'

    # ------------------- Launch arguments -------------------
    ur_type = LaunchConfiguration("ur_type")
    onrobot_type = LaunchConfiguration("onrobot_type")
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")

    ur_description_package = LaunchConfiguration("ur_description_package")
    description_file = LaunchConfiguration("description_file")
    publish_robot_description_semantic = LaunchConfiguration("publish_robot_description_semantic")
    moveit_config_package = LaunchConfiguration("moveit_config_package")
    moveit_config_file = LaunchConfiguration("moveit_config_file")
    moveit_joint_limits_file = LaunchConfiguration("moveit_joint_limits_file")
    warehouse_sqlite_path = LaunchConfiguration("warehouse_sqlite_path")
    prefix = LaunchConfiguration("prefix")
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_rviz = LaunchConfiguration("launch_rviz")
    launch_servo = LaunchConfiguration("launch_servo")
    use_camera = LaunchConfiguration("use_camera")

    # ------------------- URDF (xacro) -------------------
    joint_limit_params = PathJoinSubstitution(
        [FindPackageShare(ur_description_package), "config", ur_type.perform(context), "joint_limits.yaml"]
    )
    kinematics_params = PathJoinSubstitution(
        [FindPackageShare(ur_description_package), "config", ur_type.perform(context), "default_kinematics.yaml"]
    )
    physical_params = PathJoinSubstitution(
        [FindPackageShare(ur_description_package), "config", ur_type.perform(context), "physical_parameters.yaml"]
    )
    visual_params = PathJoinSubstitution(
        [FindPackageShare(ur_description_package), "config", ur_type.perform(context), "visual_parameters.yaml"]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("ur_onrobot_description"), "urdf", description_file]),
            " robot_ip:=", chosen_robot_ip if chosen_robot_ip else "127.0.0.1",
            " joint_limit_params:=", joint_limit_params,
            " kinematics_params:=", kinematics_params,
            " physical_params:=", physical_params,
            " visual_params:=", visual_params,
            " safety_limits:=", safety_limits,
            " safety_pos_margin:=", safety_pos_margin,
            " safety_k_position:=", safety_k_position,
            " name:=ur_onrobot",
            " ur_type:=", ur_type,
            " onrobot_type:=", onrobot_type,
            " prefix:=", prefix,
            " use_fake_hardware:=", detected_mode,
        ]
    )
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    # ------------------- SRDF -------------------
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(moveit_config_package), "srdf", moveit_config_file]),
            " name:=ur_onrobot",
            " prefix:=", prefix,
            " onrobot_type:=", onrobot_type,
        ]
    )
    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(robot_description_semantic_content, value_type=str)
    }

    # ------------------- Kinematics & Planning -------------------
    kinematics_yaml = load_yaml("ur_onrobot_moveit_config", "config/kinematics.yaml")
    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}

    robot_description_planning = {
        "robot_description_planning": load_yaml(
            moveit_config_package.perform(context),
            os.path.join("config", moveit_joint_limits_file.perform(context)),
        )
    }

    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_yaml = load_yaml("ur_onrobot_moveit_config", "config/ompl_planning.yaml")
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    # ------------------- Controllers -------------------
    controllers_yaml = load_yaml("ur_onrobot_moveit_config", "config/controllers.yaml")
    if context.perform_substitution(use_sim_time) == "true":
        controllers_yaml["scaled_joint_trajectory_controller"]["default"] = False
        controllers_yaml["joint_trajectory_controller"]["default"] = True

    moveit_controllers = {
        "moveit_simple_controller_manager": controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
        "trajectory_execution.execution_duration_monitoring": False,
    }

    # ------------------- Planning Scene Monitor -------------------
    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }
    if camera_enabled:
        sensors_3d_yaml = load_yaml("ur_onrobot_moveit_config", "config/sensors_3d.yaml")
        planning_scene_monitor_parameters.update({
            "sensors_3d": sensors_3d_yaml,
            "camera_info_topic": "/camera/depth/camera_info",
        })

    warehouse_ros_config = {
        "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
        "warehouse_host": warehouse_sqlite_path,
    }

    # ------------------- Servo -------------------
    servo_yaml = load_yaml("ur_onrobot_moveit_config", "config/ur_onrobot_servo.yaml")
    servo_params = {"moveit_servo": servo_yaml}

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
                    servo_params,
                    robot_description,
                    robot_description_semantic,
                    robot_description_kinematics,
                    planning_scene_monitor_parameters,
                    {"moveit_servo.move_group_name": "ur_onrobot_manipulator"},
                    {"move_group_name": "ur_onrobot_manipulator"},
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
        output="screen",
    )

    # ------------------- MoveGroup -------------------
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            {"publish_robot_description_semantic": publish_robot_description_semantic},
            robot_description_kinematics,
            robot_description_planning,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            {"use_sim_time": use_sim_time},
            warehouse_ros_config,
        ],
    )

    # ------------------- RViz -------------------
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(moveit_config_package), "rviz", "view_robot.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            robot_description_kinematics,
            robot_description_planning,
            warehouse_ros_config,
            planning_scene_monitor_parameters,
            {"use_sim_time": use_sim_time},
        ],
    )

    # ------------------- REAL DRIVER (only if detected_mode='false') -------------------
    real_driver = GroupAction(
        actions=[
            Node(
                package="ur_robot_driver",
                executable="ur_ros2_control_node",
                name="ur_control_node",
                output="screen",
                parameters=[
                    robot_description,
                    {"robot_ip": chosen_robot_ip},
                    {"use_fake_hardware": False},
                    {"reverse_port": 50001},
                    {"tf_prefix": prefix},
                ],
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["scaled_joint_trajectory_controller", "--controller-manager", "/controller_manager"],
                output="screen",
            ),
        ],
        condition=UnlessCondition(detected_mode),
    )

    # ------------------- FAKE DRIVER + URSim Docker (only if detected_mode='true') -------------------
    ursim_docker = ExecuteProcess(
        cmd=[
            "docker", "run", "-d", "--rm", "--name", "ursim_auto",
            "-p", "59000:59000", "-p", "29999:29999",
            "-p", "30001-30004:30001-30004",
            "universalrobots/ursim_e-series", "ur_driver"
        ],
        output="screen",
        condition=IfCondition(detected_mode),
    )

    fake_driver = Node(
        package="ur_robot_driver",
        executable="ur_ros2_control_node",
        name="ur_control_node",
        output="screen",
        parameters=[
            robot_description,
            {"use_fake_hardware": True},
            {"tf_prefix": prefix},
        ],
        condition=IfCondition(detected_mode),
    )

    fake_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["scaled_joint_trajectory_controller", "--controller-manager", "/controller_manager"],
        output="screen",
        condition=IfCondition(detected_mode),
    )

    # ------------------- Delayed startup -------------------
    delayed_nodes = TimerAction(
        period=6.0,  # give driver time to connect
        actions=[
            move_group_node,
            rviz_node,
            servo_container if context.perform_substitution(launch_servo) == "true" else None,
        ],
    )

    delayed_camera_check = TimerAction(
        period=1.5,
        actions=[OpaqueFunction(function=check_camera_and_decide)],
    )

    # ------------------- Return -------------------
    actions = [
        real_driver,
        ursim_docker,
        fake_driver,
        fake_spawner,
        delayed_camera_check,
        delayed_nodes,
    ]
    return [a for a in actions if a is not None]


# ----------------------------------------------------------------------
def generate_launch_description():
    declared_args = []

    # === Existing args (unchanged) ===
    declared_args.append(DeclareLaunchArgument("ur_type", default_value="ur5e",
        description="Type/series of used UR robot.",
        choices=["ur3","ur3e","ur5","ur5e","ur10","ur10e","ur16e","ur20","ur30"]))
    declared_args.append(DeclareLaunchArgument("onrobot_type", default_value="2fg7",
        description="Type of OnRobot gripper.", choices=["rg2","rg6","2fg7","2fg14"]))
    declared_args.append(DeclareLaunchArgument("safety_limits", default_value="true"))
    declared_args.append(DeclareLaunchArgument("safety_pos_margin", default_value="0.15"))
    declared_args.append(DeclareLaunchArgument("safety_k_position", default_value="20"))
    declared_args.append(DeclareLaunchArgument("ur_description_package", default_value="ur_description"))
    declared_args.append(DeclareLaunchArgument("description_file", default_value="ur_onrobot.urdf.xacro"))
    declared_args.append(DeclareLaunchArgument("publish_robot_description_semantic", default_value="True"))
    declared_args.append(DeclareLaunchArgument("moveit_config_package", default_value="ur_onrobot_moveit_config"))
    declared_args.append(DeclareLaunchArgument("moveit_config_file", default_value="ur_onrobot.srdf.xacro"))
    declared_args.append(DeclareLaunchArgument("moveit_joint_limits_file", default_value="joint_limits.yaml"))
    declared_args.append(DeclareLaunchArgument("warehouse_sqlite_path",
        default_value=os.path.expanduser("~/.ros/warehouse_ros.sqlite")))
    declared_args.append(DeclareLaunchArgument("use_sim_time", default_value="false"))
    declared_args.append(DeclareLaunchArgument("prefix", default_value='""'))
    declared_args.append(DeclareLaunchArgument("launch_rviz", default_value="true"))
    declared_args.append(DeclareLaunchArgument("launch_servo", default_value="true"))
    declared_args.append(DeclareLaunchArgument("use_camera", default_value="auto",
        description="OctoMap: 'auto' (detect), 'true' (force on), 'false' (force off)"))

    # === NEW: Auto-detection args ===
    declared_args.append(
        DeclareLaunchArgument(
            "robot_ip_candidates",
            default_value="192.168.1.101,192.168.1.105",
            description="Comma-separated list of robot IPs to try (in order)."
        )
    )
    declared_args.append(
        DeclareLaunchArgument(
            "connection_timeout",
            default_value="1.5",
            description="Seconds to wait for TCP connection on port 30003 per IP."
        )
    )

    # === NEW: Detected mode (set by detect_robot) ===
    declared_args.append(
        DeclareLaunchArgument(
            "detected_mode",
            default_value="true",  # fallback if detection fails
            description="Internal: 'true' for fake/URSim, 'false' for real (set by detection)."
        )
    )

    # Run detection first (yields SetLaunchConfiguration)
    detection = OpaqueFunction(function=detect_robot)

    return LaunchDescription(
        declared_args + [detection, OpaqueFunction(function=launch_setup)]
    )
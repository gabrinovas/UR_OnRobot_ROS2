# -*- coding: utf-8 -*-
# SPDX-License-Identifier: Apache-2.0
# MoveIt + UR + OnRobot (2FG7) – Real or Fake Hardware
# Basado en el launch funcional, pero con MoveIt 2

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
    AndSubstitution,
    NotSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue, ParameterFile
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess
from launch.substitutions import FindExecutable, Command


def launch_setup(context, *args, **kwargs):
    # ------------------- Launch Configurations -------------------
    ur_type = LaunchConfiguration("ur_type")
    onrobot_type = LaunchConfiguration("onrobot_type")
    robot_ip = LaunchConfiguration("robot_ip")
    tf_prefix = LaunchConfiguration("tf_prefix")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    launch_rviz = LaunchConfiguration("launch_rviz")
    launch_servo = LaunchConfiguration("launch_servo")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_camera = LaunchConfiguration("use_camera")
    controller_spawner_timeout = LaunchConfiguration("controller_spawner_timeout")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    activate_joint_controller = LaunchConfiguration("activate_joint_controller")

    # ------------------- URDF (xacro) -------------------
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("ur_onrobot_description"), "urdf", "ur_onrobot.urdf.xacro"]
            ),
            " robot_ip:=", robot_ip,
            " ur_type:=", ur_type,
            " onrobot_type:=", onrobot_type,
            " tf_prefix:=", tf_prefix,
            " name:=ur_onrobot",
            " use_fake_hardware:=", use_fake_hardware,
            " connection_type:=tcp",
            " ip_address:=", robot_ip,
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
            " prefix:=", tf_prefix,
            " onrobot_type:=", onrobot_type,
        ]
    )
    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(
            robot_description_semantic_content, value_type=str
        )
    }

    # ------------------- Kinematics -------------------
    kinematics = ParameterFile(
        PathJoinSubstitution(
            [FindPackageShare("ur_onrobot_moveit_config"), "config", "kinematics.yaml"]
        ),
        allow_substs=True,
    )

    # ------------------- Controllers -------------------
    controllers_file = ParameterFile(
        PathJoinSubstitution(
            [FindPackageShare("ur_onrobot_moveit_config"), "config", "controllers.yaml"]
        ),
        allow_substs=True,
    )

    # ------------------- Joint Limits (MoveIt) -------------------
    joint_limits = ParameterFile(
        PathJoinSubstitution(
            [FindPackageShare("ur_onrobot_moveit_config"), "config", "joint_limits.yaml"]
        ),
        allow_substs=True,
    )

    # ------------------- OMPL Planning -------------------
    ompl_planning = ParameterFile(
        PathJoinSubstitution(
            [FindPackageShare("ur_onrobot_moveit_config"), "config", "ompl_planning.yaml"]
        ),
        allow_substs=True,
    )

    # ------------------- Sensors 3D (OctoMap) -------------------
    sensors_3d = ParameterFile(
        PathJoinSubstitution(
            [FindPackageShare("ur_onrobot_moveit_config"), "config", "sensors_3d.yaml"]
        ),
        allow_substs=True,
    )

    # ------------------- Servo Config -------------------
    servo_config = ParameterFile(
        PathJoinSubstitution(
            [FindPackageShare("ur_onrobot_moveit_config"), "config", "ur_onrobot_servo.yaml"]
        ),
        allow_substs=True,
    )

    # ------------------- RViz Config -------------------
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("ur_onrobot_moveit_config"), "rviz", "view_robot.rviz"]
    )

    # ------------------- ros2_control Node -------------------
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            controllers_file,
            {
                "use_fake_hardware": use_fake_hardware,
                "robot_ip": robot_ip,
                "tf_prefix": tf_prefix,
            },
        ],
        output="screen",
    )

    # ------------------- Controller Spawner Helper -------------------
    def controller_spawner(controllers, active=True):
        inactive = ["--inactive"] if not active else []
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "--controller-manager", "/controller_manager",
                "--controller-manager-timeout", controller_spawner_timeout,
            ] + inactive + controllers,
            output="screen",
        )

    # ------------------- Controllers to Spawn -------------------
    active_controllers = [
        "joint_state_broadcaster",
        "io_and_status_controller",
        "speed_scaling_state_broadcaster",
        "force_torque_sensor_broadcaster",
        "tcp_pose_broadcaster",
        "ur_configuration_controller",
        "finger_width_trajectory_controller",
    ]

    inactive_controllers = [
        "scaled_joint_trajectory_controller",
        "joint_trajectory_controller",
    ]

    # Choose initial controller
    if initial_joint_controller.perform(context) in ["scaled_joint_trajectory_controller", "joint_trajectory_controller"]:
        if activate_joint_controller.perform(context) == "true":
            active_controllers.append(initial_joint_controller.perform(context))
        else:
            inactive_controllers.append(initial_joint_controller.perform(context))
    else:
        active_controllers.append(initial_joint_controller.perform(context))

    # Remove from inactive if active
    for c in active_controllers:
        if c in inactive_controllers:
            inactive_controllers.remove(c)

    # Remove tcp_pose_broadcaster in fake mode
    if use_fake_hardware.perform(context) == "true":
        if "tcp_pose_broadcaster" in active_controllers:
            active_controllers.remove("tcp_pose_broadcaster")

    spawners = [
        controller_spawner(active_controllers, active=True),
        controller_spawner(inactive_controllers, active=False),
    ]

    # ------------------- MoveGroup Node -------------------
    move_group_params = [
        robot_description,
        robot_description_semantic,
        kinematics,
        joint_limits,
        ompl_planning,
        {"use_sim_time": use_sim_time},
        {"moveit_controller_manager": "ros2_control"},
        {"moveit_controller_manager_name": "controller_manager"},
    ]

    # Add sensors if camera is enabled
    if use_camera.perform(context) in ["true", "auto"]:
        move_group_params.append(sensors_3d)

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=move_group_params,
    )

    # ------------------- RViz -------------------
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics,
            {"use_sim_time": use_sim_time},
        ],
        condition=IfCondition(launch_rviz),
    )

    # ------------------- Servo (Realtime Jogging) -------------------
    servo_node = Node(
        package="moveit_servo",
        executable="servo_server",
        name="servo_server",
        output="screen",
        parameters=[
            servo_config,
            robot_description,
            robot_description_semantic,
            {"move_group_name": "ur_onrobot_manipulator"},
        ],
        condition=IfCondition(launch_servo),
    )

    # ------------------- Delayed Start (MoveIt) -------------------
    delayed_nodes = TimerAction(
        period=5.0,
        actions=[
            move_group_node,
            rviz_node,
            servo_node,
        ],
    )

    # ------------------- Final Nodes List -------------------
    nodes = [
        control_node,
        *spawners,
        delayed_nodes,
    ]

    return nodes


def generate_launch_description():
    return LaunchDescription([
        # ------------------- Arguments -------------------
        DeclareLaunchArgument("ur_type", default_value="ur5e",
                              description="Type of UR robot", choices=["ur3", "ur3e", "ur5", "ur5e", "ur10", "ur10e", "ur16e", "ur20", "ur30"]),
        DeclareLaunchArgument("onrobot_type", default_value="2fg7",
                              description="Type of OnRobot gripper", choices=["rg2", "rg6", "2fg7", "2fg14", "3fg15"]),
        DeclareLaunchArgument("robot_ip", default_value="192.168.1.101",
                              description="IP address of the robot"),
        DeclareLaunchArgument("tf_prefix", default_value="",
                              description="Prefix for tf frames"),
        DeclareLaunchArgument("use_fake_hardware", default_value="false",
                              description="Use fake hardware"),
        DeclareLaunchArgument("use_sim_time", default_value="false",
                              description="Use simulation time"),
        DeclareLaunchArgument("launch_rviz", default_value="true",
                              description="Launch RViz?"),
        DeclareLaunchArgument("launch_servo", default_value="true",
                              description="Launch Servo?"),
        DeclareLaunchArgument("use_camera", default_value="auto",
                              description="'auto', 'true', or 'false' for OctoMap"),
        DeclareLaunchArgument("controller_spawner_timeout", default_value="10",
                              description="Timeout for spawners"),
        DeclareLaunchArgument("initial_joint_controller", default_value="scaled_joint_trajectory_controller",
                              choices=["scaled_joint_trajectory_controller", "joint_trajectory_controller"],
                              description="Initial joint controller"),
        DeclareLaunchArgument("activate_joint_controller", default_value="true",
                              description="Activate initial controller"),

        # ------------------- Opaque Function -------------------
        OpaqueFunction(function=launch_setup),
    ])
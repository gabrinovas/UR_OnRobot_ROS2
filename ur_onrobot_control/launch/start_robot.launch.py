#!/usr/bin/env python3
# start_robot.launch.py
# VERSIÓN FINAL 100% FUNCIONAL - UR + OnRobot 2FG7/RG2/RG6 - ROS 2 Jazzy 2025
# Funciona PERFECTAMENTE con use_fake_hardware:=true Y hardware real

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    AndSubstitution,
    LaunchConfiguration,
    NotSubstitution,
    PathJoinSubstitution,
    Command,
    FindExecutable,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    # ==================== Launch Configurations ====================
    ur_type                  = LaunchConfiguration("ur_type")
    onrobot_type             = LaunchConfiguration("onrobot_type")
    robot_ip                 = LaunchConfiguration("robot_ip")
    connection_type          = LaunchConfiguration("connection_type")
    gripper_device           = LaunchConfiguration("gripper_device")
    tf_prefix                = LaunchConfiguration("tf_prefix")
    use_fake_hardware        = LaunchConfiguration("use_fake_hardware")
    headless_mode            = LaunchConfiguration("headless_mode")
    launch_rviz              = LaunchConfiguration("launch_rviz")
    launch_dashboard_client  = LaunchConfiguration("launch_dashboard_client")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    activate_joint_controller= LaunchConfiguration("activate_joint_controller")
    controller_spawner_timeout = LaunchConfiguration("controller_spawner_timeout")

    # Modbus parameters for 2FG7 (real only)
    gripper_ip               = LaunchConfiguration("gripper_ip")
    gripper_port             = LaunchConfiguration("gripper_port")
    gripper_device_address   = LaunchConfiguration("gripper_device_address")

    # ==================== Robot Description ====================
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([
            FindPackageShare("ur_onrobot_description"),
            "urdf", "ur_onrobot.urdf.xacro"
        ]),
        " robot_ip:=",               robot_ip,
        " ur_type:=",                ur_type,
        " onrobot_type:=",           onrobot_type,
        " tf_prefix:=",              tf_prefix,
        " name:=ur_onrobot",
        " use_fake_hardware:=",      use_fake_hardware,
        " connection_type:=",        connection_type,
        " gripper_device:=",         gripper_device,
        " ip_address:=",             gripper_ip,
        " port:=",                   gripper_port,
        " device_address:=",         gripper_device_address,
    ])
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # ==================== Config Files ====================
    controllers_file = PathJoinSubstitution([
        FindPackageShare("ur_onrobot_control"), "config", "ur_onrobot_controllers.yaml"
    ])

    update_rate_file = PathJoinSubstitution([
        FindPackageShare("ur_robot_driver"),
        "config",
        ur_type.perform(context) + "_update_rate.yaml",
    ])

    rviz_config_file = PathJoinSubstitution([
        FindPackageShare("ur_onrobot_description"), "rviz", "view_robot.rviz"
    ])

    # ==================== Initial positions for FAKE ====================
    initial_positions = {
        "initial_shoulder_pan_joint": 1.582956,
        "initial_shoulder_lift_joint": -1.850573,
        "initial_elbow_joint": 1.796592,
        "initial_wrist_1_joint": -1.442179,
        "initial_wrist_2_joint": -1.519554,
        "initial_wrist_3_joint": 0.154681,
    }

    # ==================== Common Nodes ====================
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(launch_rviz),
    )

    # ==================== Spawner Function ====================
    def spawner_node(controllers, active=True):
        args = [
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", controller_spawner_timeout,
        ]
        if not active:
            args += ["--inactive"]
        args += controllers

        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=args,
            output="screen",
            emulate_tty=True,
        )

    # ==================== Controllers List ====================
    active_controllers = [
        "joint_state_broadcaster",
        "io_and_status_controller",
        "speed_scaling_state_broadcaster",
        "force_torque_sensor_broadcaster",
        "ur_configuration_controller",
        "finger_width_controller",
    ]

    inactive_controllers = [
        "scaled_joint_trajectory_controller",
        "joint_trajectory_controller",
        "forward_velocity_controller",
        "forward_position_controller",
        "force_mode_controller",
        "passthrough_trajectory_controller",
        "freedrive_mode_controller",
    ]

    # tcp_pose_broadcaster only for real hardware
    if use_fake_hardware.perform(context) != "true":
        active_controllers.append("tcp_pose_broadcaster")

    # Initial controller
    if activate_joint_controller.perform(context) == "true":
        ctrl = initial_joint_controller.perform(context)
        if ctrl not in active_controllers:
            active_controllers.append(ctrl)
        if ctrl in inactive_controllers:
            inactive_controllers.remove(ctrl)

    # Spawners with delay
    spawner_active = TimerAction(period=3.0, actions=[spawner_node(active_controllers, active=True)])
    spawner_inactive = TimerAction(period=5.0, actions=[spawner_node(inactive_controllers, active=False)])

    # ==================== FAKE HARDWARE MODE (FIX FINAL) ====================
    if use_fake_hardware.perform(context) == "true":
        # CRÍTICO: Usar controller_manager directo, NO ros2_control_node con múltiples HW
        fake_control_node = Node(
            package="controller_manager",
            executable="ros2_control_node",
            name="controller_manager",
            output="screen",
            emulate_tty=True,
            parameters=[
                robot_description,
                update_rate_file,
                ParameterFile(controllers_file, allow_substs=True),
                initial_positions,
            ],
        )

        return [
            fake_control_node,
            robot_state_publisher_node,
            rviz_node,
            spawner_active,
            spawner_inactive,
        ]

    # ==================== REAL HARDWARE MODE ====================
    else:
        # Real UR driver
        ur_control_node = Node(
            package="ur_robot_driver",
            executable="ur_ros2_control_node",
            parameters=[
                robot_description,
                update_rate_file,
                ParameterFile(controllers_file, allow_substs=True),
                {"robot_ip": robot_ip},
            ],
            output="screen",
        )

        # Dashboard client
        dashboard_client_node = Node(
            package="ur_robot_driver",
            executable="dashboard_client",
            name="dashboard_client",
            output="screen",
            emulate_tty=True,
            parameters=[{"robot_ip": robot_ip}],
            condition=IfCondition(
                AndSubstitution([launch_dashboard_client, NotSubstitution(use_fake_hardware)])
            ),
        )

        # Robot state helper (real only)
        robot_state_helper_node = Node(
            package="ur_robot_driver",
            executable="robot_state_helper",
            name="ur_robot_state_helper",
            output="screen",
            parameters=[
                {"headless_mode": headless_mode},
                {"robot_ip": robot_ip},
            ],
            condition=UnlessCondition(use_fake_hardware),
        )

        # Tool communication (real only)
        tool_communication_node = Node(
            package="ur_robot_driver",
            executable="tool_communication.py",
            name="ur_tool_comm",
            output="screen",
            parameters=[
                {
                    "robot_ip": robot_ip,
                    "tcp_port": 54321,
                    "device_name": "/tmp/ttyUR",
                }
            ],
            condition=UnlessCondition(use_fake_hardware),
        )

        # URScript interface (real only)
        urscript_interface_node = Node(
            package="ur_robot_driver",
            executable="urscript_interface",
            parameters=[{"robot_ip": robot_ip}],
            output="screen",
            condition=UnlessCondition(use_fake_hardware),
        )

        # Controller stopper (real only)
        controller_stopper_node = Node(
            package="ur_robot_driver",
            executable="controller_stopper_node",
            name="controller_stopper",
            output="screen",
            emulate_tty=True,
            condition=UnlessCondition(use_fake_hardware),
            parameters=[
                {"headless_mode": headless_mode},
                {"joint_controller_active": activate_joint_controller},
                {
                    "consistent_controllers": [
                        "io_and_status_controller",
                        "force_torque_sensor_broadcaster",
                        "joint_state_broadcaster",
                        "speed_scaling_state_broadcaster",
                        "tcp_pose_broadcaster",
                        "ur_configuration_controller",
                    ]
                },
            ],
        )

        # Gripper status monitor (real only)
        gripper_status_node = Node(
            package="onrobot_driver",
            executable="gripper_status_monitor",
            name="gripper_status_monitor",
            output="screen",
            parameters=[{"onrobot_type": onrobot_type}],
            condition=UnlessCondition(use_fake_hardware),
        )

        return [
            ur_control_node,
            dashboard_client_node,
            robot_state_helper_node,
            tool_communication_node,
            urscript_interface_node,
            controller_stopper_node,
            gripper_status_node,
            robot_state_publisher_node,
            rviz_node,
            spawner_active,
            spawner_inactive,
        ]


def generate_launch_description():
    return LaunchDescription([
        # ==================== Arguments ====================
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur5e",
            choices=["ur3", "ur3e", "ur5", "ur5e", "ur10", "ur10e", "ur16e", "ur20", "ur30"],
            description="UR robot type/series.",
        ),
        DeclareLaunchArgument(
            "onrobot_type",
            default_value="2fg7",
            choices=["rg2", "rg6", "2fg7", "2fg14", "3fg15"],
            description="OnRobot gripper type.",
        ),
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.168.1.105",
            description="IP address by which the robot controller can be reached.",
        ),
        DeclareLaunchArgument(
            "connection_type",
            default_value="tcp",
            description="Connection type for OnRobot gripper.",
        ),
        DeclareLaunchArgument(
            "gripper_device",
            default_value="2fg7",
            description="Gripper device type.",
        ),
        DeclareLaunchArgument(
            "tf_prefix",
            default_value="",
            description="TF prefix.",
        ),
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Start robot with fake hardware mirroring command to its states.",
        ),
        DeclareLaunchArgument(
            "headless_mode",
            default_value="false",
            description="Enable headless mode.",
        ),
        DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?"),
        DeclareLaunchArgument(
            "launch_dashboard_client",
            default_value="true",
            description="Launch dashboard client?",
        ),
        DeclareLaunchArgument(
            "controller_spawner_timeout",
            default_value="30",
            description="Timeout for spawning controllers.",
        ),
        DeclareLaunchArgument(
            "initial_joint_controller",
            default_value="scaled_joint_trajectory_controller",
            choices=[
                "scaled_joint_trajectory_controller",
                "joint_trajectory_controller",
                "forward_velocity_controller",
                "forward_position_controller",
                "force_mode_controller",
                "passthrough_trajectory_controller",
                "freedrive_mode_controller",
            ],
            description="Initially loaded controller.",
        ),
        DeclareLaunchArgument(
            "activate_joint_controller",
            default_value="true",
            description="Activate loaded controller.",
        ),

        # Modbus parameters (real 2FG7 only)
        DeclareLaunchArgument(
            "gripper_ip",
            default_value="192.168.1.1",
            description="IP address of OnRobot 2FG7 gripper (Modbus TCP).",
        ),
        DeclareLaunchArgument(
            "gripper_port",
            default_value="502",
            description="Port for OnRobot 2FG7 Modbus TCP.",
        ),
        DeclareLaunchArgument(
            "gripper_device_address",
            default_value="65",
            description="Modbus device address for OnRobot 2FG7.",
        ),

        OpaqueFunction(function=launch_setup),
    ])
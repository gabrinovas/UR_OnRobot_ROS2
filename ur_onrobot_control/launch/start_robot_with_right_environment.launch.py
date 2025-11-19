#!/usr/bin/env python3
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    AndSubstitution,
    Command,
    FindExecutable,
    LaunchConfiguration,
    NotSubstitution,
    PathJoinSubstitution,
)


def launch_setup(context, *args, **kwargs):
    # Argumentos
    ur_type = LaunchConfiguration("ur_type")
    onrobot_type = LaunchConfiguration("onrobot_type")
    robot_ip = LaunchConfiguration("robot_ip")
    tf_prefix = LaunchConfiguration("tf_prefix")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    headless_mode = LaunchConfiguration("headless_mode")
    launch_rviz = LaunchConfiguration("launch_rviz")
    launch_dashboard_client = LaunchConfiguration("launch_dashboard_client")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    activate_joint_controller = LaunchConfiguration("activate_joint_controller")
    controller_spawner_timeout = LaunchConfiguration("controller_spawner_timeout")

    # Parámetros Modbus gripper (2FG7)
    gripper_ip = LaunchConfiguration("gripper_ip")
    gripper_port = LaunchConfiguration("gripper_port")
    gripper_device_address = LaunchConfiguration("gripper_device_address")

    # === robot_description con entorno ===
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([
                FindPackageShare("ur_onrobot_control"),
                "urdf",
                "robot_with_right_environment.urdf.xacro"   # ← TU ENTORNO
            ]),
            " robot_ip:=", robot_ip,
            " ur_type:=", ur_type,
            " onrobot_type:=", onrobot_type,
            " tf_prefix:=", tf_prefix,
            " use_fake_hardware:=", use_fake_hardware,
            " connection_type:=tcp",
            " ip_address:=", gripper_ip,
            " port:=", gripper_port,
            " device_address:=", gripper_device_address,
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # Config files
    controllers_file = PathJoinSubstitution([
        FindPackageShare("ur_onrobot_control"),
        "config",
        "ur_onrobot_with_environment_controllers.yaml"
    ])

    rviz_config_file = PathJoinSubstitution([
        FindPackageShare("ur_workspace_description"),
        "rviz",
        "view_platform.rviz"
    ])

    update_rate_config_file = PathJoinSubstitution([
        FindPackageShare("ur_robot_driver"),
        "config",
        ur_type.perform(context) + "_update_rate.yaml",
    ])

    # Posición inicial realista (igual que tu start_robot.launch.py bueno)
    initial_positions_params = {
        "initial_shoulder_pan_joint": 1.582956,
        "initial_shoulder_lift_joint": -1.850573,
        "initial_elbow_joint": 1.796592,
        "initial_wrist_1_joint": -1.442179,
        "initial_wrist_2_joint": -1.519554,
        "initial_wrist_3_joint": 0.154681,
    }

    # Nodos principales
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            update_rate_config_file,
            ParameterFile(controllers_file, allow_substs=True),
            initial_positions_params,
        ],
        output="screen",
        condition=IfCondition(use_fake_hardware),
    )

    ur_control_node = Node(
        package="ur_robot_driver",
        executable="ur_ros2_control_node",
        parameters=[
            robot_description,
            update_rate_config_file,
            ParameterFile(controllers_file, allow_substs=True),
            {"robot_ip": robot_ip},
        ],
        output="screen",
        condition=UnlessCondition(use_fake_hardware),
    )

    dashboard_client_node = Node(
        package="ur_robot_driver",
        executable="dashboard_client",
        name="dashboard_client",
        output="screen",
        emulate_tty=True,
        parameters=[{"robot_ip": robot_ip}],
        condition=IfCondition(
            AndSubstitution(launch_dashboard_client, NotSubstitution(use_fake_hardware))
        ),
    )

    robot_state_helper_node = Node(
        package="ur_robot_driver",
        executable="robot_state_helper",
        name="ur_robot_state_helper",
        output="screen",
        parameters=[
            {"headless_mode": headless_mode},
            {"robot_ip": robot_ip},
        ],
    )

    tool_communication_node = Node(
        package="ur_robot_driver",
        executable="tool_communication.py",
        name="ur_tool_comm",
        output="screen",
        parameters={
            "robot_ip": robot_ip,
            "tcp_port": 54321,
            "device_name": "/tmp/ttyUR",
        },
    )

    urscript_interface = Node(
        package="ur_robot_driver",
        executable="urscript_interface",
        parameters=[{"robot_ip": robot_ip}],
        output="screen",
    )

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
            {"consistent_controllers": [
                "io_and_status_controller",
                "force_torque_sensor_broadcaster",
                "joint_state_broadcaster",
                "speed_scaling_state_broadcaster",
                "tcp_pose_broadcaster",
                "ur_configuration_controller",
                "finger_width_controller",
            ]},
        ],
    )

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

    gripper_status_node = Node(
        package="onrobot_driver",
        executable="gripper_status_monitor",
        name="gripper_status_monitor",
        output="screen",
        parameters=[{"onrobot_type": onrobot_type}],
        condition=UnlessCondition(use_fake_hardware),
    )

    # === Spawners (idéntico al start_robot.launch.py) ===
    def controller_spawner(controllers, active=True):
        inactive_flags = ["--inactive"] if not active else []
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "--controller-manager", "/controller_manager",
                "--controller-manager-timeout", controller_spawner_timeout,
            ] + inactive_flags + controllers,
        )

    active_controllers = [
        "joint_state_broadcaster",
        "io_and_status_controller",
        "speed_scaling_state_broadcaster",
        "force_torque_sensor_broadcaster",
        "tcp_pose_broadcaster",
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

    if activate_joint_controller.perform(context) == "true":
        ctrl = initial_joint_controller.perform(context)
        if ctrl in inactive_controllers:
            active_controllers.append(ctrl)
            inactive_controllers.remove(ctrl)

    if use_fake_hardware.perform(context) == "true":
        if "tcp_pose_broadcaster" in active_controllers:
            active_controllers.remove("tcp_pose_broadcaster")

    spawner_nodes = [
        controller_spawner(active_controllers),
        controller_spawner(inactive_controllers, active=False),
    ]

    return [
        control_node,
        ur_control_node,
        dashboard_client_node,
        robot_state_helper_node,
        tool_communication_node,
        controller_stopper_node,
        urscript_interface,
        robot_state_publisher_node,
        rviz_node,
        gripper_status_node,
    ] + spawner_nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("ur_type", default_value="ur5e",
                              description="Type/series of used UR robot.", choices=["ur3", "ur3e", "ur5", "ur5e", "ur10", "ur10e", "ur16e", "ur20", "ur30"]),
        DeclareLaunchArgument("onrobot_type", default_value="2fg7",
                              description="Type of OnRobot gripper.", choices=["rg2", "rg6", "2fg7", "2fg14", "3fg15"]),
        DeclareLaunchArgument("robot_ip", default_value="192.168.1.105",
                              description="IP address of the robot."),
        DeclareLaunchArgument("tf_prefix", default_value="", description="TF prefix for multi-robot setups."),
        DeclareLaunchArgument("use_fake_hardware", default_value="false", description="Use fake hardware."),
        DeclareLaunchArgument("headless_mode", default_value="false", description="Headless mode."),
        DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?"),
        DeclareLaunchArgument("launch_dashboard_client", default_value="true", description="Launch dashboard client?"),
        DeclareLaunchArgument("initial_joint_controller", default_value="scaled_joint_trajectory_controller",
                              description="Initially active controller."),
        DeclareLaunchArgument("activate_joint_controller", default_value="true"),
        DeclareLaunchArgument("controller_spawner_timeout", default_value="10"),

        # Parámetros Modbus 2FG7 (exactamente como tu start_robot.launch.py)
        DeclareLaunchArgument("gripper_ip", default_value="192.168.1.1"),
        DeclareLaunchArgument("gripper_port", default_value="502"),
        DeclareLaunchArgument("gripper_device_address", default_value="65"),

        OpaqueFunction(function=launch_setup)
    ])
# start_robot.launch.py
# VERSIÓN DEFINITIVA – FUNCIONA EN REAL Y FAKE – Noviembre 2025
# UR + OnRobot 2FG7/RG2/RG6 (cualquier combinación)

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    Command,
    FindExecutable,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    # ==================== Parámetros de lanzamiento ====================
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
    activate_joint_controller = LaunchConfiguration("activate_joint_controller")
    controller_spawner_timeout = LaunchConfiguration("controller_spawner_timeout")

    # Parámetros Modbus (solo reales)
    gripper_ip               = LaunchConfiguration("gripper_ip")
    gripper_port             = LaunchConfiguration("gripper_port")
    gripper_device_address   = LaunchConfiguration("gripper_device_address")

    # ==================== Robot Description (xacro) ====================
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([
                FindPackageShare("ur_onrobot_description"),
                "urdf",
                "ur_onrobot.urdf.xacro"
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
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # ==================== Archivos de configuración ====================
    controllers_file = PathJoinSubstitution([
        FindPackageShare("ur_onrobot_control"),
        "config",
        "ur_onrobot_controllers.yaml"
    ])

    update_rate_file = PathJoinSubstitution([
        FindPackageShare("ur_robot_driver"),
        "config",
        ur_type.perform(context) + "_update_rate.yaml",
    ])

    rviz_config_file = PathJoinSubstitution([
        FindPackageShare("ur_onrobot_description"),
        "rviz",
        "view_robot.rviz"
    ])

    # Posiciones iniciales para fake hardware
    initial_positions = {
        "initial_shoulder_pan_joint": 1.582956,
        "initial_shoulder_lift_joint": -1.850573,
        "initial_elbow_joint":  1.796592,
        "initial_wrist_1_joint": -1.442179,
        "initial_wrist_2_joint": -1.519554,
        "initial_wrist_3_joint":  0.154681,
    }

    # ==================== Nodos comunes ====================
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

    # ==================== Spawner helper ====================
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

    # ==================== Lista de controladores ====================
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

    # tcp_pose_broadcaster solo en hardware real
    if use_fake_hardware.perform(context) != "true":
        active_controllers.append("tcp_pose_broadcaster")

    # Controlador inicial activo si se pide
    if activate_joint_controller.perform(context) == "true":
        ctrl = initial_joint_controller.perform(context)
        if ctrl not in active_controllers:
            active_controllers.append(ctrl)
        if ctrl in inactive_controllers:
            inactive_controllers.remove(ctrl)

    spawner_active   = TimerAction(period=3.0,  actions=[spawner_node(active_controllers,   active=True)])
    spawner_inactive = TimerAction(period=5.0, actions=[spawner_node(inactive_controllers, active=False)])

    # ==================== MODO FAKE HARDWARE (¡EL TRUCO FINAL!) ====================
    if use_fake_hardware.perform(context) == "true":
        control_node = Node(
            package="controller_manager",
            executable="ros2_control_node",
            name="controller_manager",
            namespace="/",                                          # ¡¡CLAVE!! Servicios en namespace global
            output="screen",
            emulate_tty=True,
            parameters=[
                robot_description,
                update_rate_file,
                ParameterFile(controllers_file, allow_substs=True),
                initial_positions,
            ],
            remappings=[
                ("~/robot_description", "/robot_description"),      # Elimina warning deprecated
            ],
        )

        return [
            control_node,
            robot_state_publisher_node,
            rviz_node,
            spawner_active,
            spawner_inactive,
        ]

    # ==================== MODO HARDWARE REAL ====================
    else:
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

        dashboard_client_node = Node(
            package="ur_robot_driver",
            executable="dashboard_client",
            name="dashboard_client",
            output="screen",
            emulate_tty=True,
            parameters=[{"robot_ip": robot_ip}],
            condition=IfCondition(launch_dashboard_client),
        )

        robot_state_helper_node = Node(
            package="ur_robot_driver",
            executable="robot_state_helper",
            output="screen",
            parameters=[
                {"headless_mode": headless_mode},
                {"robot_ip": robot_ip},
            ],
        )

        tool_communication_node = Node(
            package="ur_robot_driver",
            executable="tool_communication.py",
            output="screen",
            parameters={
                "robot_ip": robot_ip,
                "tcp_port": 54321,
                "device_name": "/tmp/ttyUR",
            },
        )

        urscript_interface_node = Node(
            package="ur_robot_driver",
            executable="urscript_interface",
            output="screen",
            parameters=[{"robot_ip": robot_ip}],
        )

        controller_stopper_node = Node(
            package="ur_robot_driver",
            executable="controller_stopper_node",
            name="controller_stopper",
            output="screen",
            emulate_tty=True,
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
                ]},
            ],
        )

        gripper_status_node = Node(
            package="onrobot_driver",
            executable="gripper_status_monitor",
            output="screen",
            parameters=[{"onrobot_type": onrobot_type}],
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
        DeclareLaunchArgument("ur_type", default_value="ur5e",
            choices=["ur3","ur3e","ur5","ur5e","ur10","ur10e","ur16e","ur20","ur30"],
            description="Tipo de robot UR"),
        DeclareLaunchArgument("onrobot_type", default_value="2fg7",
            choices=["rg2","rg6","2fg7","2fg14","3fg15"],
            description="Tipo de gripper OnRobot"),
        DeclareLaunchArgument("robot_ip", default_value="192.168.1.105", description="IP del robot UR"),
        DeclareLaunchArgument("connection_type", default_value="tcp", description="Conexión del gripper"),
        DeclareLaunchArgument("gripper_device", default_value="2fg7", description="Dispositivo gripper"),
        DeclareLaunchArgument("tf_prefix", default_value="", description="Prefijo TF"),
        DeclareLaunchArgument("use_fake_hardware", default_value="false", description="Modo simulación"),
        DeclareLaunchArgument("headless_mode", default_value="false", description="Modo sin GUI en robot"),
        DeclareLaunchArgument("launch_rviz", default_value="true", description="Lanzar RViz"),
        DeclareLaunchArgument("launch_dashboard_client", default_value="true", description="Lanzar dashboard client"),
        DeclareLaunchArgument("initial_joint_controller", default_value="scaled_joint_trajectory_controller",
            description="Controlador inicial"),
        DeclareLaunchArgument("activate_joint_controller", default_value="true", description="Activar controlador inicial"),
        DeclareLaunchArgument("controller_spawner_timeout", default_value="30", description="Timeout spawner (segundos)"),

        # Modbus para 2FG7 real
        DeclareLaunchArgument("gripper_ip", default_value="192.168.1.1", description="IP del gripper"),
        DeclareLaunchArgument("gripper_port", default_value="502", description="Puerto Modbus"),
        DeclareLaunchArgument("gripper_device_address", default_value="65", description="Dirección Modbus"),

        OpaqueFunction(function=launch_setup)
    ])
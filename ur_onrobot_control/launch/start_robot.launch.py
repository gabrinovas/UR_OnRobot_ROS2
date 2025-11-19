# launch/start_robot.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    FindExecutable,
    Command,
    AndSubstitution,
    NotSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    # ====================== Launch Configurations ======================
    ur_type                  = LaunchConfiguration("ur_type")
    onrobot_type             = LaunchConfiguration("onrobot_type")
    robot_ip                 = LaunchConfiguration("robot_ip")
    tf_prefix                = LaunchConfiguration("tf_prefix")
    use_fake_hardware        = LaunchConfiguration("use_fake_hardware")
    robot_pos               = LaunchConfiguration("robot_pos")   # nuevo!
    headless_mode            = LaunchConfiguration("headless_mode")
    launch_rviz              = LaunchConfiguration("launch_rviz")
    launch_dashboard_client  = LaunchConfiguration("launch_dashboard_client")
    controller_spawner_timeout = LaunchConfiguration("controller_spawner_timeout")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    activate_joint_controller = LaunchConfiguration("activate_joint_controller")

    # UR description parameters
    joint_limits_parameters_file = LaunchConfiguration("joint_limits_parameters_file")
    kinematics_parameters_file   = LaunchConfiguration("kinematics_parameters_file")
    physical_parameters_file     = LaunchConfiguration("physical_parameters_file")
    visual_parameters_file       = LaunchConfiguration("visual_parameters_file")
    script_filename              = LaunchConfiguration("script_filename")
    output_recipe_filename       = LaunchConfiguration("output_recipe_filename")
    input_recipe_filename        = LaunchConfiguration("input_recipe_filename")

    # Gripper Modbus parameters
    connection_type        = LaunchConfiguration("connection_type")
    gripper_ip             = LaunchConfiguration("gripper_ip")
    gripper_port           = LaunchConfiguration("gripper_port")
    gripper_device_address = LaunchConfiguration("gripper_device_address")
    gripper_device         = LaunchConfiguration("gripper_device")

    # ====================== Selección según robot_pos ======================
    if robot_pos == "left":
        # Robot a la izquierda → entorno espejo
        xacro_file       = PathJoinSubstitution(
            [FindPackageShare("ur_onrobot_control"), "urdf", "left_robot_with_environment.urdf.xacro"]
        )
        controllers_yaml = PathJoinSubstitution(
            [FindPackageShare("ur_onrobot_control"), "config", "ur_onrobot_with_environment_controllers.yaml"]
        )
        rviz_file        = PathJoinSubstitution(
            [FindPackageShare("ur_workspace_description"), "rviz", "view_platform.rviz"]
        )
        has_environment  = True

    elif robot_pos == "right":
        # Robot a la derecha → entorno principal (el habitual)
        xacro_file       = PathJoinSubstitution(
            [FindPackageShare("ur_onrobot_control"), "urdf", "right_robot_with_environment.urdf.xacro"]
        )
        controllers_yaml = PathJoinSubstitution(
            [FindPackageShare("ur_onrobot_control"), "config", "ur_onrobot_with_environment_controllers.yaml"]
        )
        rviz_file        = PathJoinSubstitution(
            [FindPackageShare("ur_workspace_description"), "rviz", "view_platform.rviz"]
        )
        has_environment  = True

    else:  # "none" o cualquier otro valor → solo robot + gripper
        xacro_file       = PathJoinSubstitution(
            [FindPackageShare("ur_onrobot_description"), "urdf", "ur_onrobot.urdf.xacro"]
        )
        controllers_yaml = PathJoinSubstitution(
            [FindPackageShare("ur_onrobot_control"), "config", "ur_onrobot_controllers.yaml"]
        )
        rviz_file        = PathJoinSubstitution(
            [FindPackageShare("ur_onrobot_description"), "rviz", "view_robot.rviz"]
        )
        has_environment  = False

    # ====================== Robot description (xacro) ======================
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            xacro_file,
            " robot_ip:=", robot_ip,
            " ur_type:=", ur_type,
            " onrobot_type:=", onrobot_type,
            " tf_prefix:=", tf_prefix,
            " use_fake_hardware:=", use_fake_hardware,
            " joint_limits_parameters_file:=", joint_limits_parameters_file,
            " kinematics_parameters_file:=", kinematics_parameters_file,
            " physical_parameters_file:=", physical_parameters_file,
            " visual_parameters_file:=", visual_parameters_file,
            " script_filename:=", script_filename,
            " output_recipe_filename:=", output_recipe_filename,
            " input_recipe_filename:=", input_recipe_filename,
            " connection_type:=", connection_type,
            " ip_address:=", gripper_ip,
            " port:=", gripper_port,
            " device_address:=", gripper_device_address,
            " device:=", gripper_device,
        ]
    )

    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # ====================== Update rate config ======================
    update_rate_config_file = PathJoinSubstitution(
        [FindPackageShare("ur_robot_driver"), "config", ur_type.perform(context) + "_update_rate.yaml"]
    )

    # ====================== Posiciones iniciales SOLO en simulación ======================
    initial_positions = {
        "initial_shoulder_pan_joint": 0.0,
        "initial_shoulder_lift_joint": -1.57,
        "initial_elbow_joint": 1.57,
        "initial_wrist_1_joint": -1.57,
        "initial_wrist_2_joint": -1.57,
        "initial_wrist_3_joint": 0.0,
    }

    # ====================== Nodos principales ======================
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            update_rate_config_file,
            ParameterFile(controllers_yaml, allow_substs=True),
            initial_positions,                       # ← solo afecta en fake hardware
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
            ParameterFile(controllers_yaml, allow_substs=True),
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
        parameters=[{"headless_mode": headless_mode}],
    )

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
            ] + (["environment_joint_state_broadcaster"] if has_environment else [])},
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
        arguments=["-d", rviz_file],
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

    # ====================== Spawner de controladores ======================
    def spawner(controllers, active=True):
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "--controller-manager", "/controller_manager",
                "--controller-manager-timeout", controller_spawner_timeout,
            ] + (["--inactive"] if not active else []) + controllers,
        )

    # Controladores siempre activos
    always_active = [
        "joint_state_broadcaster",
        "io_and_status_controller",
        "speed_scaling_state_broadcaster",
        "force_torque_sensor_broadcaster",
        "ur_configuration_controller",
        "finger_width_controller",
        "scaled_joint_trajectory_controller",
    ]

    if has_environment:
        always_active.append("environment_joint_state_broadcaster")
    if use_fake_hardware.perform(context) != "true":
        always_active.append("tcp_pose_broadcaster")

    # Controladores condicionales (se cargan inactivos)
    conditional_controllers = [
        "scaled_joint_trajectory_controller",
        "joint_trajectory_controller",
        "forward_velocity_controller",
        "forward_position_controller",
        "force_mode_controller",
        "passthrough_trajectory_controller",
        "freedrive_mode_controller",
    ]

    # Activar el controlador inicial si se pide
    if activate_joint_controller.perform(context) == "true":
        init_ctrl = initial_joint_controller.perform(context)
        if init_ctrl in conditional_controllers:
            always_active.append(init_ctrl)
            conditional_controllers.remove(init_ctrl)

    controller_spawners = [
        spawner(always_active),
        spawner(conditional_controllers, active=False),
    ]

    # ====================== Lista final de nodos ======================
    nodes_to_start = [
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
    ] + controller_spawners

    return nodes_to_start


def generate_launch_description():
    return LaunchDescription([
        # ========== Argumentos ==========
        DeclareLaunchArgument("ur_type", default_value="ur5e",
                              description="Type/series of used UR robot.",
                              choices=["ur3", "ur3e", "ur5", "ur5e", "ur10", "ur10e", "ur16e", "ur20", "ur30"]),

        DeclareLaunchArgument("onrobot_type", default_value="2fg7",
                              description="Type of OnRobot gripper.",
                              choices=["rg2", "rg6", "2fg7", "2fg14", "3fg15"]),

        DeclareLaunchArgument("robot_ip", default_value="192.168.1.101",
                              description="IP address of the robot."),

        DeclareLaunchArgument("tf_prefix", default_value="",
                              description="Prefix for joint names (multi-robot)."),

        DeclareLaunchArgument("use_fake_hardware", default_value="false",
                              description="Use fake hardware (simulation)."),

        DeclareLaunchArgument("robot_pos", default_value="right",
                              description="Posición física del robot respecto al entorno.",
                              choices=["left", "right", "none"]),

        DeclareLaunchArgument("headless_mode", default_value="false",
                              description="Enable headless mode for robot control."),

        DeclareLaunchArgument("launch_rviz", default_value="true",
                              description="Launch RViz?"),

        DeclareLaunchArgument("launch_dashboard_client", default_value="true",
                              description="Launch UR Dashboard Client?"),

        DeclareLaunchArgument("controller_spawner_timeout", default_value="10",
                              description="Timeout for spawning controllers."),

        DeclareLaunchArgument("initial_joint_controller", default_value="scaled_joint_trajectory_controller",
                              choices=["scaled_joint_trajectory_controller", "joint_trajectory_controller",
                                       "forward_velocity_controller", "forward_position_controller",
                                       "freedrive_mode_controller", "passthrough_trajectory_controller"],
                              description="Initially active joint controller."),

        DeclareLaunchArgument("activate_joint_controller", default_value="true",
                              description="Activate the initial joint controller on start."),

        # UR description parameters (con defaults correctos)
        DeclareLaunchArgument("joint_limits_parameters_file",
            default_value=PathJoinSubstitution([FindPackageShare("ur_description"), "config",
                                                LaunchConfiguration("ur_type"), "joint_limits.yaml"])),
        DeclareLaunchArgument("kinematics_parameters_file",
            default_value=PathJoinSubstitution([FindPackageShare("ur_description"), "config",
                                                LaunchConfiguration("ur_type"), "default_kinematics.yaml"])),
        DeclareLaunchArgument("physical_parameters_file",
            default_value=PathJoinSubstitution([FindPackageShare("ur_description"), "config",
                                                LaunchConfiguration("ur_type"), "physical_parameters.yaml"])),
        DeclareLaunchArgument("visual_parameters_file",
            default_value=PathJoinSubstitution([FindPackageShare("ur_description"), "config",
                                                LaunchConfiguration("ur_type"), "visual_parameters.yaml"])),
        DeclareLaunchArgument("script_filename",
            default_value=PathJoinSubstitution([FindPackageShare("ur_robot_driver"), "resources", "ros_control.urscript"])),
        DeclareLaunchArgument("output_recipe_filename",
            default_value=PathJoinSubstitution([FindPackageShare("ur_robot_driver"), "resources", "rtde_output_recipe.txt"])),
        DeclareLaunchArgument("input_recipe_filename",
            default_value=PathJoinSubstitution([FindPackageShare("ur_robot_driver"), "resources", "rtde_input_recipe.txt"])),

        # Gripper parameters
        DeclareLaunchArgument("connection_type", default_value="tcp", choices=["tcp", "serial"],
                              description="Connection type for OnRobot gripper."),
        DeclareLaunchArgument("gripper_ip", default_value="192.168.1.1"),
        DeclareLaunchArgument("gripper_port", default_value="502"),
        DeclareLaunchArgument("gripper_device_address", default_value="65"),
        DeclareLaunchArgument("gripper_device", default_value="/tmp/ttyUR"),

        OpaqueFunction(function=launch_setup)
    ])
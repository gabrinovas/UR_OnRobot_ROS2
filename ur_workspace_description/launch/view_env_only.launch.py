# filename: view_env_only.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    # ===================================================================
    # Argumentos de lanzamiento
    # ===================================================================
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_pos",
            default_value="left",
            choices=["left", "right"],
            description="Posición del robot: 'left' o 'right'. Decide qué entorno cargar."
        )
    )

    # Argumentos opcionales (no se usan aquí, pero se mantienen por compatibilidad)
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur5e",
            description="Tipo de robot UR (no usado en este launch, solo por compatibilidad)"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "onrobot_type",
            default_value="2fg7",
            description="Tipo de gripper OnRobot (no usado en este launch)"
        )
    )

    # ===================================================================
    # Configuraciones
    # ===================================================================
    robot_pos = LaunchConfiguration("robot_pos")

    # Selección dinámica del archivo xacro según robot_pos
    environment_xacro_file = PathJoinSubstitution(
        [
            FindPackageShare("ur_workspace_description"),
            "urdf",
            Command([
                "echo Only_environment_for_",
                robot_pos,
                "_robot.urdf.xacro"
            ])
        ]
    )

    # Procesamos el xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            environment_xacro_file
        ]
    )

    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # ===================================================================
    # Nodos comunes
    # ===================================================================
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("ur_workspace_description"), "rviz", "view_platform.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
    )

    # ===================================================================
    # LaunchDescription final
    # ===================================================================
    nodes = [
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node
    ]

    return LaunchDescription(declared_arguments + nodes)
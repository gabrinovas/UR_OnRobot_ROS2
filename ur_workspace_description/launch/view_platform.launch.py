from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition, UnlessCondition

def launch_setup(context, *args, **kwargs):
    # Initialize Arguments
    ur_type = LaunchConfiguration("ur_type")
    onrobot_type = LaunchConfiguration("onrobot_type")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    
    # MODBUS parameters for 2FG7 gripper
    gripper_ip = LaunchConfiguration("gripper_ip", default="192.168.1.1")
    gripper_port = LaunchConfiguration("gripper_port", default="502")
    gripper_device_address = LaunchConfiguration("gripper_device_address", default="65")

    # Platform description with MODBUS parameters
    platform_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare('ur_workspace_description'), "urdf", 'platform.urdf.xacro']),
            " ",
            "ur_type:=",
            ur_type,
            " ",
            "onrobot_type:=",
            onrobot_type,
            " ",
            "use_fake_hardware:=",
            use_fake_hardware,
            " ",
            # MODBUS parameters for 2FG7
            "connection_type:=tcp",
            " ",
            "ip_address:=",
            gripper_ip,
            " ",
            "port:=",
            gripper_port,
            " ",
            "device_address:=",
            gripper_device_address,
            " ",
        ]
    )
    
    # Use ParameterValue to properly handle the command output
    platform_description = {'robot_description': ParameterValue(platform_description_content, value_type=str)}

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[platform_description],
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare('ur_workspace_description'), "rviz", "view_platform.rviz"]
    )
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    # Add joint state publisher for manual joint control in visualization
    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        condition=IfCondition(use_fake_hardware)
    )

    # For real hardware, we need a joint state publisher that doesn't require GUI
    joint_state_publisher_non_gui_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        condition=UnlessCondition(use_fake_hardware)
    )

    return [
        robot_state_publisher_node, 
        joint_state_publisher_node,
        joint_state_publisher_non_gui_node,
        rviz_node
    ]

def generate_launch_description():
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur5e",
            description="Type of UR robot",
            choices=["ur3", "ur3e", "ur5", "ur5e", "ur10", "ur10e", "ur16e", "ur20"],
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "onrobot_type",
            default_value="2fg7",
            description="Type of OnRobot gripper",
            choices=["rg2", "rg6", "2fg7", "2fg14"],
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="true",
            description="Use fake hardware",
        )
    )

    # MODBUS parameters for 2FG7 gripper
    declared_arguments.append(
        DeclareLaunchArgument(
            "gripper_ip",
            default_value="192.168.1.1",
            description="IP address of the OnRobot 2FG7 gripper (Modbus TCP server).",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gripper_port",
            default_value="502",
            description="Port number for the OnRobot 2FG7 gripper Modbus TCP connection.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gripper_device_address",
            default_value="65",
            description="Modbus device address for the OnRobot 2FG7 gripper.",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
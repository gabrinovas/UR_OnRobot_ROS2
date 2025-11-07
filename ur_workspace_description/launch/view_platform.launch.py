from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

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

    ur_type = LaunchConfiguration("ur_type")
    onrobot_type = LaunchConfiguration("onrobot_type")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")

    # Platform description - REMOVE use_fake_hardware parameter from xacro command
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
        ]
    )
    
    platform_description = {'robot_description': platform_description_content}

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

    return LaunchDescription(declared_arguments + [robot_state_publisher_node, rviz_node])
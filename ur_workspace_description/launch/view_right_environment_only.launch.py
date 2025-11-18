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
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "onrobot_type", 
            default_value="2fg7",
            description="Type of OnRobot gripper",
        )
    )

    ur_type = LaunchConfiguration("ur_type")
    onrobot_type = LaunchConfiguration("onrobot_type")

    # Simple environment description without robot for testing
    environment_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare('ur_workspace_description'), "urdf", 'environment_right.urdf.xacro']),
        ]
    )
    
    environment_description = {'robot_description': ParameterValue(environment_description_content, value_type=str)}

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[environment_description],
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

    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
    )

    return LaunchDescription(declared_arguments + [
        robot_state_publisher_node, 
        joint_state_publisher_node,
        rviz_node
    ])
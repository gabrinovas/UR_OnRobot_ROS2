#!/usr/bin/env python3

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)


def load_yaml(package_name, file_path):
    """Load YAML file from package."""
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except Exception as e:
        print(f"Failed to load YAML file {absolute_file_path}: {e}")
        return None


def launch_setup(context, *args, **kwargs):
    # Initialize Arguments
    ur_type = LaunchConfiguration("ur_type")
    onrobot_type = LaunchConfiguration("onrobot_type")
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")

    # General arguments
    ur_description_package = LaunchConfiguration("ur_description_package")
    description_file = LaunchConfiguration("description_file")
    moveit_config_package = LaunchConfiguration("moveit_config_package")
    moveit_config_file = LaunchConfiguration("moveit_config_file")
    prefix = LaunchConfiguration("prefix")
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_rviz = LaunchConfiguration("launch_rviz")
    launch_servo = LaunchConfiguration("launch_servo")

    # Get actual values from context
    onrobot_type_str = onrobot_type.perform(context)
    prefix_str = prefix.perform(context)

    joint_limit_params = PathJoinSubstitution(
        [FindPackageShare(ur_description_package), "config", ur_type, "joint_limits.yaml"]
    )
    kinematics_params = PathJoinSubstitution(
        [FindPackageShare(ur_description_package), "config", ur_type, "default_kinematics.yaml"]
    )
    physical_params = PathJoinSubstitution(
        [FindPackageShare(ur_description_package), "config", ur_type, "physical_parameters.yaml"]
    )
    visual_params = PathJoinSubstitution(
        [FindPackageShare(ur_description_package), "config", ur_type, "visual_parameters.yaml"]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("ur_onrobot_description"), "urdf", description_file]),
            " ",
            "robot_ip:=xxx.yyy.zzz.www",
            " ",
            "joint_limit_params:=",
            joint_limit_params,
            " ",
            "kinematics_params:=",
            kinematics_params,
            " ",
            "physical_params:=",
            physical_params,
            " ",
            "visual_params:=",
            visual_params,
            " ",
            "safety_limits:=",
            safety_limits,
            " ",
            "safety_pos_margin:=",
            safety_pos_margin,
            " ",
            "safety_k_position:=",
            safety_k_position,
            " ",
            "name:=",
            "ur_onrobot",
            " ",
            "ur_type:=",
            ur_type,
            " ",
            "onrobot_type:=",
            onrobot_type,
            " ",
            "script_filename:=ros_control.urscript",
            " ",
            "input_recipe_filename:=rtde_input_recipe.txt",
            " ",
            "output_recipe_filename:=rtde_output_recipe.txt",
            " ",
            "prefix:=",
            prefix,
            " ",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # MoveIt Configuration
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(moveit_config_package), "srdf", moveit_config_file]
            ),
            " ",
            "name:=",
            "ur_onrobot",
            " ",
            "prefix:=",
            prefix,
            " ",
            "onrobot_type:=",
            onrobot_type,
            " ",
        ]
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_content
    }

    # Load MoveIt config YAML files as dictionaries
    moveit_config_dir = get_package_share_directory(moveit_config_package.perform(context))
    
    kinematics_yaml = load_yaml(moveit_config_package.perform(context), "config/kinematics.yaml")
    joint_limits_yaml = load_yaml(moveit_config_package.perform(context), "config/joint_limits.yaml")
    moveit_controllers_yaml = load_yaml(moveit_config_package.perform(context), "config/moveit_controllers.yaml")
    ompl_yaml = load_yaml(moveit_config_package.perform(context), "config/ompl_planning.yaml")
    planning_pipelines_yaml = load_yaml(moveit_config_package.perform(context), "config/planning_pipelines.yaml")

    # Debug output
    print("\n" + "="*60)
    print("DEBUG: YAML Files Loading Status")
    print("="*60)
    print(f"kinematics.yaml: {'LOADED' if kinematics_yaml else 'FAILED'}")
    print(f"joint_limits.yaml: {'LOADED' if joint_limits_yaml else 'FAILED'}")
    print(f"moveit_controllers.yaml: {'LOADED' if moveit_controllers_yaml else 'FAILED'}")
    print(f"ompl_planning.yaml: {'LOADED' if ompl_yaml else 'FAILED'}")
    
    if moveit_controllers_yaml:
        controllers = moveit_controllers_yaml.get('ros__parameters', {}).get('moveit_simple_controller_manager', {})
        if 'ros__parameters' in controllers:
            controller_names = controllers['ros__parameters'].get('controller_names', [])
            print(f"Controllers found: {controller_names}")
            if not controller_names:
                print("WARNING: controller_names is empty!")
        else:
            print("ERROR: No ros__parameters in moveit_simple_controller_manager")
    print("="*60 + "\n")

    # Combine all parameters
    move_group_params = []
    
    # Add robot description and semantic
    move_group_params.append(robot_description)
    move_group_params.append(robot_description_semantic)
    
    # Add YAML configurations if they exist
    if kinematics_yaml:
        move_group_params.append(kinematics_yaml)
    if joint_limits_yaml:
        move_group_params.append(joint_limits_yaml)
    if moveit_controllers_yaml:
        move_group_params.append(moveit_controllers_yaml)
    if ompl_yaml:
        move_group_params.append(ompl_yaml)
    if planning_pipelines_yaml:
        move_group_params.append(planning_pipelines_yaml)
    
    # Add essential MoveIt parameters
    move_group_params.append({
        "use_sim_time": use_sim_time,
        "publish_robot_description_semantic": True,
        "publish_robot_description": True,
        "moveit_manage_controllers": True,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
        "planning_plugin": "ompl_interface/OMPLPlanner",
        "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
        "start_state_max_bounds_error": 0.1,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
        "trajectory_execution.execution_duration_monitoring": False,
    })

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=move_group_params,
    )

    # rviz with moveit configuration
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
            {"use_sim_time": use_sim_time},
        ],
    )

    # Servo node - DISABLED BY DEFAULT
    # Note: MoveIt Servo has known issues with default parameters
    servo_node = Node(
        package="moveit_servo",
        condition=IfCondition(launch_servo),
        executable="servo_node_main",
        parameters=[
            {
                "move_group_name": "ur_onrobot_manipulator",
                "planning_frame": "base_link",
                "ee_frame_name": "gripper_tcp",
                "robot_link_command_frame": "base_link",
                "command_in_type": "speed_units",
                "command_out_type": "std_msgs/Float64MultiArray",
                "publish_joint_positions": True,
                "publish_joint_velocities": False,
                "check_collisions": True,
                "collision_check_rate": 5.0,
                "scale.linear": 0.6,
                "scale.rotational": 0.3,
                "scale.joint": 0.01,
                "low_pass_filter_coeff": 10.0,
                "joint_topic": "/joint_states",
                "command_out_topic": "/scaled_joint_trajectory_controller/joint_trajectory",
                "status_topic": "/servo_node/status",
                "cartesian_command_in_topic": "/servo_node/delta_twist_cmds",
                "joint_command_in_topic": "/servo_node/delta_joint_cmds",
                "use_sim_time": use_sim_time,
            },
            robot_description,
            robot_description_semantic,
        ],
        output="screen",
    )

    nodes_to_start = [move_group_node, rviz_node, servo_node]

    return nodes_to_start


def generate_launch_description():

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            description="Type/series of used UR robot.",
            choices=["ur3", "ur3e", "ur5", "ur5e", "ur10", "ur10e", "ur16e", "ur20", "ur30"],
            default_value="ur5e",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "onrobot_type",
            description="Type of the OnRobot gripper.",
            choices=["2fg7", "3fg15", "rg2", "rg6"],
            default_value="2fg7",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_limits",
            default_value="true",
            description="Enables the safety limits controller if true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_pos_margin",
            default_value="0.15",
            description="The margin to lower and upper limits in the safety controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_k_position",
            default_value="20",
            description="k-position factor in the safety controller.",
        )
    )
    # General arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_description_package",
            default_value="ur_description",
            description="Description package with robot URDF/XACRO files. Usually the argument "
            "is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="ur_onrobot.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_package",
            default_value="ur_onrobot_moveit_config",
            description="MoveIt config package with robot SRDF/XACRO files. Usually the argument "
            "is not set, it enables use of a custom moveit config.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_file",
            default_value="ur_onrobot.srdf.xacro",
            description="MoveIt SRDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Make MoveIt to use simulation time. This is needed for the trajectory planing in simulation.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for "
            "multi-robot setup. If changed than also joint names in the controllers' configuration "
            "have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?")
    )
    declared_arguments.append(
        DeclareLaunchArgument("launch_servo", default_value="false", description="Launch Servo?")
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])

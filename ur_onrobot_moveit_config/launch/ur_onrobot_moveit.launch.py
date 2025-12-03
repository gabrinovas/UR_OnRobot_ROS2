# Copyright (c) 2021 PickNik, Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the {copyright_holder} nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

#
# Author: Denis Stogl

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
    """Load yaml file from package."""
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        print(f"Warning: Could not load YAML file: {absolute_file_path}")
        return None


def load_gripper_joint_limits(onrobot_type_str, prefix_str):
    """Load joint limits for specific gripper type."""
    try:
        all_limits = load_yaml("ur_onrobot_moveit_config", "config/joint_limits_grippers.yaml")
        
        if not all_limits:
            print(f"Warning: Could not load gripper joint limits file")
            return {}
        
        # Select the appropriate gripper limits
        gripper_key = f"joint_limits_{onrobot_type_str}"
        if gripper_key in all_limits:
            gripper_limits = all_limits[gripper_key].copy()
            
            # Add prefix to joint names if needed
            if prefix_str and prefix_str != '""':
                prefixed_limits = {}
                for joint_name, limits in gripper_limits.items():
                    prefixed_joint_name = f"{prefix_str}{joint_name}"
                    prefixed_limits[prefixed_joint_name] = limits
                return prefixed_limits
            return gripper_limits
        
        print(f"Warning: No joint limits found for gripper type '{onrobot_type_str}'")
        return {}
    except Exception as e:
        print(f"Warning: Could not load gripper joint limits: {e}")
        return {}


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
    moveit_joint_limits_file = LaunchConfiguration("moveit_joint_limits_file")
    moveit_config_file = LaunchConfiguration("moveit_config_file")
    warehouse_sqlite_path = LaunchConfiguration("warehouse_sqlite_path")
    prefix = LaunchConfiguration("prefix")
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_rviz = LaunchConfiguration("launch_rviz")
    launch_servo = LaunchConfiguration("launch_servo")

    # Get actual values from context
    onrobot_type_str = onrobot_type.perform(context)
    prefix_str = prefix.perform(context)
    
    # Load gripper-specific joint limits
    gripper_limits = load_gripper_joint_limits(onrobot_type_str, prefix_str)

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

    robot_description_kinematics = PathJoinSubstitution(
        [FindPackageShare(moveit_config_package), "config", "kinematics.yaml"]
    )

    # Load base joint limits
    robot_description_planning = {
        "robot_description_planning": load_yaml(
            moveit_config_package.perform(context),
            os.path.join("config", moveit_joint_limits_file.perform(context))
        )
    }
    
    # Merge gripper-specific limits with base limits
    if gripper_limits and robot_description_planning["robot_description_planning"]:
        if "joint_limits" not in robot_description_planning["robot_description_planning"]:
            robot_description_planning["robot_description_planning"]["joint_limits"] = {}
        
        robot_description_planning["robot_description_planning"]["joint_limits"].update(gripper_limits)

    # Planning Configuration
    ompl_planning_pipeline_config = {
        "ompl": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization
                default_planner_request_adapters/FixWorkspaceBounds
                default_planner_request_adapters/FixStartStateBounds
                default_planner_request_adapters/FixStartStateCollision
                default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }
    
    # Load OMPL planning configuration
    ompl_planning_yaml = load_yaml("ur_onrobot_moveit_config", "config/ompl_planning.yaml")
    if ompl_planning_yaml:
        ompl_planning_pipeline_config["ompl"].update(ompl_planning_yaml)

    # MoveIt Controllers Configuration
    moveit_controllers_config = load_yaml("ur_onrobot_moveit_config", "config/moveit_controllers.yaml")
    
    if not moveit_controllers_config:
        # Configuración por defecto si no se puede cargar el archivo
        moveit_controllers_config = {
            "moveit_simple_controller_manager": {
                "ros__parameters": {
                    "controller_names": [
                        "scaled_joint_trajectory_controller",
                        "joint_trajectory_controller",
                        "finger_width_trajectory_controller"
                    ],
                    "scaled_joint_trajectory_controller": {
                        "type": "follow_joint_trajectory/FollowJointTrajectory",
                        "joints": [
                            "shoulder_pan_joint",
                            "shoulder_lift_joint",
                            "elbow_joint",
                            "wrist_1_joint",
                            "wrist_2_joint",
                            "wrist_3_joint"
                        ]
                    },
                    "joint_trajectory_controller": {
                        "type": "follow_joint_trajectory/FollowJointTrajectory",
                        "joints": [
                            "shoulder_pan_joint",
                            "shoulder_lift_joint",
                            "elbow_joint",
                            "wrist_1_joint",
                            "wrist_2_joint",
                            "wrist_3_joint"
                        ]
                    },
                    "finger_width_trajectory_controller": {
                        "type": "follow_joint_trajectory/FollowJointTrajectory",
                        "joints": ["finger_width"]
                    }
                }
            }
        }

    # Configuración de controladores de MoveIt
    moveit_controllers = {
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager"
    }
    
    # Agregar la configuración específica
    if "moveit_simple_controller_manager" in moveit_controllers_config:
        moveit_controllers.update(moveit_controllers_config)

    # Configuración de ejecución de trayectorias
    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
        "trajectory_execution.execution_duration_monitoring": False,
    }

    # Configuración del monitor de escena de planificación
    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            {"use_sim_time": use_sim_time},
        ],
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
            robot_description_kinematics,
            {"use_sim_time": use_sim_time},
        ],
    )

    # Servo node for realtime control
    servo_yaml = load_yaml("ur_onrobot_moveit_config", "config/ur_onrobot_servo.yaml")
    servo_params = {"moveit_servo": servo_yaml}
    servo_node = Node(
        package="moveit_servo",
        condition=IfCondition(launch_servo),
        executable="servo_node_main",
        parameters=[
            servo_params,
            robot_description,
            robot_description_semantic,
            {"use_sim_time": use_sim_time},
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
            "moveit_joint_limits_file",
            default_value="joint_limits.yaml",
            description="MoveIt joint limits that augment or override the values from the URDF robot_description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "warehouse_sqlite_path",
            default_value=os.path.expanduser("~/.ros/warehouse_ros.sqlite"),
            description="Path where the warehouse database should be stored",
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
        DeclareLaunchArgument("launch_servo", default_value="true", description="Launch Servo?")
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])

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
import subprocess
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    FindExecutable,
    Command,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ur_onrobot_moveit_config.launch_common import load_yaml


# Global state
camera_detected = False
camera_enabled = False


def check_camera_and_decide(context):
    """Check for RealSense and decide whether to enable OctoMap."""
    global camera_detected, camera_enabled
    use_camera_val = LaunchConfiguration("use_camera").perform(context)

    # Detect camera
    try:
        result = subprocess.run(
            ["ros2", "topic", "list"], capture_output=True, text=True, timeout=2
        )
        topics = result.stdout.splitlines()
        camera_detected = "/camera/depth/image_rect_raw" in topics
    except:
        camera_detected = False

    # Apply hybrid logic
    if use_camera_val == "true":
        camera_enabled = True
        print("[INFO] use_camera:=true → Forcing OctoMap ON")
    elif use_camera_val == "false":
        camera_enabled = False
        print("[INFO] use_camera:=false → OctoMap disabled")
    else:
        camera_enabled = camera_detected
        print(f"[INFO] {'RealSense D435i detected → enabling OctoMap' if camera_detected else 'No RealSense → running without OctoMap'}")

    return []


def launch_setup(context, *args, **kwargs):
    global camera_enabled

    # === Launch Arguments ===
    ur_type = LaunchConfiguration("ur_type")
    onrobot_type = LaunchConfiguration("onrobot_type")
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")

    # General arguments
    ur_description_package = LaunchConfiguration("ur_description_package")
    description_file = LaunchConfiguration("description_file")
    _publish_robot_description_semantic = LaunchConfiguration("publish_robot_description_semantic")
    moveit_config_package = LaunchConfiguration("moveit_config_package")
    moveit_joint_limits_file = LaunchConfiguration("moveit_joint_limits_file")
    moveit_config_file = LaunchConfiguration("moveit_config_file")
    warehouse_sqlite_path = LaunchConfiguration("warehouse_sqlite_path")
    prefix = LaunchConfiguration("prefix")
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_rviz = LaunchConfiguration("launch_rviz")
    launch_servo = LaunchConfiguration("launch_servo")

    # === URDF ===
    joint_limit_params = PathJoinSubstitution(
        [FindPackageShare(ur_description_package), "config", ur_type.perform(context), "joint_limits.yaml"]
    )
    kinematics_params = PathJoinSubstitution(
        [FindPackageShare(ur_description_package), "config", ur_type.perform(context), "default_kinematics.yaml"]
    )
    physical_params = PathJoinSubstitution(
        [FindPackageShare(ur_description_package), "config", ur_type.perform(context), "physical_parameters.yaml"]
    )
    visual_params = PathJoinSubstitution(
        [FindPackageShare(ur_description_package), "config", ur_type.perform(context), "visual_parameters.yaml"]
    )

    # Robot Description (URDF)
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("ur_onrobot_description"), "urdf", description_file]),
            " robot_ip:=xxx.yyy.zzz.www",
            " joint_limit_params:=", joint_limit_params,
            " kinematics_params:=", kinematics_params,
            " physical_params:=", physical_params,
            " visual_params:=", visual_params,
            " safety_limits:=", safety_limits,
            " safety_pos_margin:=", safety_pos_margin,
            " safety_k_position:=", safety_k_position,
            " name:=ur_onrobot",
            " ur_type:=", ur_type,
            " onrobot_type:=", onrobot_type,
            " prefix:=", prefix,
            " use_fake_hardware:=true",
        ]
    )
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    # === SRDF ===
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(moveit_config_package), "srdf", moveit_config_file]),
            " name:=ur_onrobot",
            " prefix:=", prefix,
            " onrobot_type:=", onrobot_type,
        ]
    )
    robot_description_semantic = {"robot_description_semantic": ParameterValue(robot_description_semantic_content, value_type=str)}

    publish_robot_description_semantic = {"publish_robot_description_semantic": _publish_robot_description_semantic}

    # === Kinematics ===
    kinematics_yaml = load_yaml("ur_onrobot_moveit_config", "config/kinematics.yaml")
    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}

    # === Joint Limits ===
    robot_description_planning = {
        "robot_description_planning": load_yaml(
            str(moveit_config_package.perform(context)),
            os.path.join("config", str(moveit_joint_limits_file.perform(context))),
        )
    }

    # === OMPL ===
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_yaml = load_yaml("ur_onrobot_moveit_config", "config/ompl_planning.yaml")
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    # === Controllers ===
    controllers_yaml = load_yaml("ur_onrobot_moveit_config", "config/controllers.yaml")
    if context.perform_substitution(use_sim_time) == "true":
        controllers_yaml["scaled_joint_trajectory_controller"]["default"] = False
        controllers_yaml["joint_trajectory_controller"]["default"] = True

    moveit_controllers = {
        "moveit_simple_controller_manager": controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
        "trajectory_execution.execution_duration_monitoring": False,
    }

    # === Planning Scene Monitor ===
    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    # === Conditionally Add RealSense OctoMap ===
    if camera_enabled:
        sensors_3d_yaml = load_yaml("ur_onrobot_moveit_config", "config/sensors_3d.yaml")
        planning_scene_monitor_parameters.update({
            "sensors_3d": sensors_3d_yaml,
            "camera_info_topic": "/camera/depth/camera_info",
        })

    warehouse_ros_config = {
        "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
        "warehouse_host": warehouse_sqlite_path,
    }

    # === Servo Node ===
    servo_yaml = load_yaml("ur_onrobot_moveit_config", "config/ur_onrobot_servo.yaml")
    servo_params = {"moveit_servo": servo_yaml}

    servo_node = Node(
        package="moveit_servo",
        condition=IfCondition(launch_servo),
        executable="servo_node_main",
        name="servo_node",
        output="screen",
        parameters=[
            servo_params,
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            planning_scene_monitor_parameters,
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    # === MoveGroup Node ===
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            publish_robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            {"use_sim_time": use_sim_time},
            warehouse_ros_config,
        ],
    )

    # === RViz ===
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
            ompl_planning_pipeline_config,
            robot_description_kinematics,
            robot_description_planning,
            warehouse_ros_config,
            planning_scene_monitor_parameters,
            {"use_sim_time": use_sim_time},
        ],
    )

    # === Delayed Camera Check ===
    delayed_check = TimerAction(
        period=1.5,
        actions=[OpaqueFunction(function=check_camera_and_decide)]
    )

    return [
        delayed_check,
        move_group_node,
        rviz_node,
        servo_node,
    ]

def generate_launch_description():
    declared_arguments = []

    # Robot & Gripper
    declared_arguments.append(
        DeclareLaunchArgument("ur_type", default_value="ur5e",
                              description="Type/series of used UR robot.",
                              choices=["ur3", "ur3e", "ur5", "ur5e", "ur10", "ur10e", "ur16e", "ur20", "ur30"])
    )
    declared_arguments.append(
        DeclareLaunchArgument("onrobot_type", default_value="2fg7",
                              description="Type of OnRobot gripper.",
                              choices=["rg2", "rg6", "2fg7", "2fg14"])
    )

    # Safety
    declared_arguments.append(DeclareLaunchArgument("safety_limits", default_value="true"))
    declared_arguments.append(DeclareLaunchArgument("safety_pos_margin", default_value="0.15"))
    declared_arguments.append(DeclareLaunchArgument("safety_k_position", default_value="20"))

    # Descriptions
    declared_arguments.append(DeclareLaunchArgument("ur_description_package", default_value="ur_description"))
    declared_arguments.append(DeclareLaunchArgument("description_file", default_value="ur_onrobot.urdf.xacro"))
    declared_arguments.append(DeclareLaunchArgument("publish_robot_description_semantic", default_value="True"))
    declared_arguments.append(DeclareLaunchArgument("moveit_config_package", default_value="ur_onrobot_moveit_config"))
    declared_arguments.append(DeclareLaunchArgument("moveit_config_file", default_value="ur_onrobot.srdf.xacro"))
    declared_arguments.append(DeclareLaunchArgument("moveit_joint_limits_file", default_value="joint_limits.yaml"))

    # Misc
    declared_arguments.append(DeclareLaunchArgument("warehouse_sqlite_path",
        default_value=os.path.expanduser("~/.ros/warehouse_ros.sqlite")))
    declared_arguments.append(DeclareLaunchArgument("use_sim_time", default_value="false"))
    declared_arguments.append(DeclareLaunchArgument("prefix", default_value='""'))
    declared_arguments.append(DeclareLaunchArgument("launch_rviz", default_value="true"))
    declared_arguments.append(DeclareLaunchArgument("launch_servo", default_value="true"))

    # === Hybrid Camera ===
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_camera",
            default_value="auto",
            description="OctoMap: 'auto' (detect), 'true' (force on), 'false' (force off)"
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])

# UR_OnRobot_ROS2
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

<img src=doc/images/ur_onrobot.gif width=30%>

ROS2 integration for Universal Robots e-Series mounted with OnRobot grippers — combined description, control and MoveIt! configuration.

## Quick overview
Repository provides three main packages:
- ur_onrobot_description: combined URDF / xacro robot description (UR + OnRobot).
- ur_onrobot_control: launch files and controllers for robot + gripper.
- ur_onrobot_moveit_config: MoveIt! configuration and launches for planning.

Key files
- ur_onrobot_description/urdf/ur_onrobot.urdf.xacro
- ur_onrobot_description/launch/view_robot.launch.py
- ur_onrobot_control/launch/start_robot.launch.py
- ur_onrobot_control/launch/ur_control.launch.py
- ur_onrobot_control/scripts/joint_state_merger.py
- ur_onrobot_moveit_config/launch/ur_onrobot_moveit.launch.py
- LICENSE

## Features
- Single combined robot description (UR arm + OnRobot grippers).
- Launch stacks for running robot driver + controllers.
- MoveIt! configs for planning with the combined robot.
- Joint state merger to include gripper finger width in /joint_states.

## Dependencies
- ROS 2 (targeted for Humble/Foxy-compatible stacks; adjust for your distro).
- Universal_Robots_ROS2_Driver (humble branch recommended).
- OnRobot ROS2 Driver and OnRobot description (see required.repos).
- libnet1-dev (for Modbus TCP/Serial).
- Modbus C++ library (included as a submodule via required.repos).

## Installation

1. Clone into your ROS2 workspace `src/`:
   ```sh
   git clone https://github.com/tonydle/UR_OnRobot_ROS2.git src/ur_onrobot
   ```
2. Import external repositories referenced by this repo:
   ```sh
   vcs import src --input src/ur_onrobot/required.repos --recursive
   ```
3. Install system deps (example for Modbus):
   ```sh
   sudo apt update
   sudo apt install libnet1-dev
   ```
4. Install ROS deps:
   ```sh
   rosdep install -y --from-paths src --ignore-src
   ```
5. Build and source:
   ```sh
   colcon build --symlink-install
   source install/setup.bash
   ```

## Hardware setup (OnRobot / UR)
1. Connect the OnRobot Quick Changer / gripper to the UR Tool I/O.
2. On the UR teach pendant install the RS485 Daemon URCap:
   - Releases and instructions: https://github.com/UniversalRobots/Universal_Robots_ToolComm_Forwarder_URCap
   - Note: conflict may exist with some URCaps (see issue tracker).
3. Configure Tool I/O in the robot installation settings (baud, parity, voltage, outputs).
   Example recommended settings (Tool I/O -> Controlled by: User, Baud 1M, Parity Even, Tool Voltage 24V).

## Usage

View URDF in RViz:
```sh
ros2 launch ur_onrobot_description view_robot.launch.py ur_type:=ur5e onrobot_type:=2fg7
```

Start robot (driver + controllers):
```sh
ros2 launch ur_onrobot_control start_robot.launch.py ur_type:=ur5e onrobot_type:=2fg7 robot_ip:=<robot_ip>
```
Common launch args:
- use_fake_hardware (default: false) — simulate hardware.
- launch_rviz (default: true) — start RViz.
- tf_prefix (default: "") — TF frame prefix.
- environment (auto/left/right/basic) — selects URDF/environment.

Start MoveIt! (planning + RViz):
```sh
ros2 launch ur_onrobot_moveit_config ur_onrobot_moveit.launch.py ur_type:=ur5e onrobot_type:=2fg7
```

Get merged joint states (includes finger_width in metres):
```sh
ros2 topic echo /joint_states
```

Control gripper (JointGroupPositionController named finger_width_controller):
```sh
ros2 topic pub --once /finger_width_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.05]}"
```

## Troubleshooting
- If autodetect fails, pass robot_ip:=<ip> to start_robot.launch.py.
- To run purely simulated, set use_fake_hardware:=true.
- If MoveIt! shows a different URDF, verify the environment, description_package and description_file args in the launch files.
- Check logs in the ROS2 output and the terminal for driver/Modbus errors.

## Development notes
- Use colcon with --symlink-install for iterative development.
- Launch files use xacro and standard launch PathJoinSubstitution to find assets inside packages.
- Modify MoveIt! settings under ur_onrobot_moveit_config/config and relaunch MoveIt!.
- The joint_state_merger script merges arm + gripper for a complete /joint_states message.

## Authors
[Tony Le] (http://github.com/tonydle) \
[Gabriel Novas] (http://github.com/gabrinovas)

See LICENSE for copyright and terms.

## License
This project is licensed under the MIT License. See LICENSE in the repository root.
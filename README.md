# UR_OnRobot_ROS2
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

<img src=doc/images/ur_onrobot.gif width=30%>

ROS 2 (Humble) integration of Universal Robots arms with OnRobot grippers: description of the robot in its cell,
launch files and controllers, safety watchdog and MoveIt configuration.

| Package | Content |
|---|---|
| `ur_workspace_description` | the cells: what surrounds the robot, one data file per cell |
| `ur_onrobot_description` | description of the arm with its gripper, standing in a cell |
| `ur_onrobot_control` | launch files, controllers, joint state merger, safety watchdog, selector and teleoperation |
| `ur_onrobot_moveit_config` | MoveIt 2 and MoveIt Servo |

The grippers come from [OnRobot_ROS2_Description](https://github.com/gabrinovas/OnRobot_ROS2_Description)
(geometry) and [OnRobot_ROS2_Driver](https://github.com/gabrinovas/OnRobot_ROS2_Driver) (hardware).

## Installation

```sh
git clone https://github.com/gabrinovas/UR_OnRobot_ROS2.git src/UR_OnRobot_ROS2
vcs import src --input src/UR_OnRobot_ROS2/required.repos --recursive
sudo apt install libnet1-dev
rosdep install -y --from-paths src --ignore-src
colcon build
source install/setup.bash
```

### Hardware

1. Connect the OnRobot quick changer or gripper to the tool connector of the robot, or to a Compute Box.
2. For the tool connector, install the RS485 daemon URCap on the pendant
   (https://github.com/UniversalRobots/Universal_Robots_ToolComm_Forwarder_URCap) and set the tool I/O to:
   controlled by user, 1 Mbit/s, even parity, 24 V.

## Running a cell

```sh
# simulated
ros2 launch ur_onrobot_control ur_onrobot_bringup.launch.py cell:=right onrobot_type:=2fg7
# real robot (needs the robot addresses file, see below)
ros2 launch ur_onrobot_control ur_onrobot_bringup.launch.py use_fake_hardware:=false cell:=right onrobot_type:=2fg7
# choosing in a terminal window first
ros2 launch ur_onrobot_control start_robot.launch.py
```

| Argument | Meaning |
|---|---|
| `use_fake_hardware` | `true` simulated arm and gripper, `false` the real robot |
| `cell` | cell the robot stands in: `basic`, `left`, `right` (the files in `ur_workspace_description/config/cells`) |
| `ur_type` | model of the robot (the files in `ur_onrobot_control/config/update_rate`) |
| `onrobot_type` | gripper: `2fg7`, `3fg15`, `vgc10` |
| `robot_ip` | address of the real robot; empty chooses among the addresses of the cell (see "Robot addresses") |
| `tf_prefix` | prefix of the joint names |
| `launch_onrobot` | run the gripper in a controller manager of its own (`/onrobot`) |
| `launch_rviz`, `rviz_config` | start RViz, and with which layout |
| `launch_watchdog` | start the safety watchdog (simulated or real robot) |
| `load_freedrive_controller` | load the hand guiding controller (see below) |

What an argument takes when it is not given is in `ur_onrobot_control/config/bringup.yaml`.

Other launch files:

| Launch file | Purpose |
|---|---|
| `ur_onrobot_control ur_control.launch.py` | the driver of the arm alone |
| `ur_onrobot_control ur_onrobot_watchdog.launch.py` | the safety watchdog alone |
| `ur_onrobot_control foxglove_bridge.launch.py` | WebSocket bridge for Foxglove Studio (`ws://<host>:8765`); a panel layout is in `ur_onrobot_control/config/foxglove` |
| `ur_onrobot_moveit_config ur_onrobot_moveit.launch.py` | MoveIt (move_group and RViz) |
| `ur_onrobot_moveit_config ur_onrobot_servo.launch.py` | MoveIt Servo |
| `ur_onrobot_description view_robot.launch.py` | look at the robot with its gripper in a cell |
| `ur_workspace_description view_cell.launch.py` | look at the furniture of a cell |

Keyboard jogging with MoveIt Servo:
`ros2 run ur_onrobot_control servo_keyboard_teleop.py --onrobot-type 2fg7`.

### Robot addresses

The addresses of the robots of an installation are not in this repository. They are in a file of your own,
`robot_addresses.yaml`, whose format is `ur_workspace_description/config/robot_addresses.template.yaml`:

```yaml
calibration_directory: ~/robot_calibration
addresses:
  <address>:
    cells: [right, basic]            # cells this robot can be used in
    kinematics_calibration: null     # or a file, relative to calibration_directory
default:
  right: <address>                   # one default per cell
gripper_compute_box:
  right: <address>                   # Compute Box of the gripper of the cell (gripper over tcp)
```

Copy the template, fill it in and either set the environment variable `UR_ONROBOT_ROBOT_ADDRESSES` to its path
or save it as `ur_workspace_description/config/robot_addresses.yaml` (git ignores it) and build again.

With the real robot and no `robot_ip`, the launch asks every address of the cell on the network (one ping):
the one that answers is used when exactly one does, otherwise the default of the cell. The log says which
address was chosen and why. A `robot_ip` that the file does not list for the cell is used with a warning.
The gripper driver is given the Compute Box of the cell; a real gripper over tcp does not start without it.
Launching the real robot without the file is an error; the simulation does not read it.

### Robot calibration

Every UR arm leaves the factory with its own kinematics, slightly different from the nominal ones of its
model. The robot itself always moves with its real kinematics; without them ROS computes the pose of the tool
(TF, RViz, MoveIt) a few millimetres off. To use them, extract them once from each robot (it does not move):

```sh
ros2 run ur_calibration calibration_correction --ros-args \
  -p robot_ip:=<address> -p output_filename:=<calibration_directory>/ur5e/<address>.yaml
```

and name the file in `kinematics_calibration` of that address. With the real robot the control node, the robot
state publisher and MoveIt then use it; an address without calibration uses the nominal kinematics and the
launch warns about it; a calibration named in the file that does not exist is an error. The simulation always
uses the nominal kinematics. `ur_control.launch.py`, launched alone, takes the file in `kinematics_params_file`.

### Update rate

The controller manager of the arm runs at 100 Hz: `update_rate: 100` of
`ur_onrobot_control/config/controllers/controller_manager.yaml`, which is loaded after the file of the model
(`config/update_rate/<ur_type>.yaml`, 500 Hz for the e-Series) and therefore wins. To run at the rate of the
model, delete that line. Check then the warning `update_rate is 0` of the scaled trajectory controller, which
is related: it samples the trajectory with the measured period, and its own `update_rate` is not set.

### Topics

- `/joint_states`: the arm. `/onrobot/joint_states`: the gripper. `/merged_joint_states`: both, used by the robot
  state publisher and by MoveIt.
- Arm: `/scaled_joint_trajectory_controller/follow_joint_trajectory` (and `joint_trajectory_controller`).
- Gripper: see OnRobot_ROS2_Driver.

### Freedrive controller

The freedrive (hand guiding) controller of `ur_controllers` is not loaded unless asked for. It starts a logging
thread when it is configured and does not stop it when the node shuts down, so while it is loaded the control node
can hang or crash when the cell is stopped (about one stop in three to five). With
`load_freedrive_controller:=true` the controller is loaded (inactive) and the launch gives the control node
`freedrive_stop_timeout` seconds (2 unless changed) to stop before terminating it; by then the hardware has been
deactivated.

## Configuration

No value is written in the code: every package has a `config/` folder, and a missing file or key stops the
launch with an error that starts with `[Fail-Fast]` and names it.

| File | Content |
|---|---|
| `ur_workspace_description/config/cells/<cell>.yaml` | a cell: orientation of its robot, its furniture, its MoveIt home pose |
| `ur_workspace_description/config/robot_addresses.template.yaml` | format of the private file with the addresses and calibrations of the robots |
| `ur_onrobot_description/config/ur_onrobot_description.yaml` | files the description is built from |
| `ur_onrobot_control/config/bringup.yaml` | arguments of the cell bringup and what simulation and real robot fix |
| `ur_onrobot_control/config/ur_control.yaml` | arguments of the arm driver and its lists of controllers |
| `ur_onrobot_control/config/controllers/` | parameters of the controllers |
| `ur_onrobot_control/config/update_rate/<ur_type>.yaml` | one file per supported model of robot |
| `ur_onrobot_control/config/safety_watchdog.yaml` | what the gripper does on a protective stop |
| `ur_onrobot_control/config/joint_state_merger.yaml`, `servo_keyboard_teleop.yaml`, `foxglove.yaml` | the other nodes |
| `ur_onrobot_moveit_config/config/moveit_launch.yaml` | arguments and files of the MoveIt launch files |

### Adding a cell

Add `ur_workspace_description/config/cells/<name>.yaml`. A cell with the furniture of the existing ones (two
tables, a conveyor, safety barriers) needs nothing else: it gives the positions. A cell with other furniture
also brings its own environment file (a xacro with a macro `cell_environment`) and its file of collision pairs
for MoveIt, and names both in its YAML. No launch file or description has to be edited: they list the files
of that folder. The address of its robot goes in the robot addresses file.

### Adding a gripper

See OnRobot_ROS2_Description and OnRobot_ROS2_Driver; here only
`ur_onrobot_moveit_config/srdf/grippers/<gripper>.srdf.xacro` is needed, for MoveIt.

## Tests

`colcon test --packages-select ur_workspace_description ur_onrobot_description ur_onrobot_control
ur_onrobot_moveit_config` runs the linters, the checks of the configuration files and descriptions, the tests
of the watchdog and a launch test that starts a simulated cell.

## Authors

Tony Le (http://github.com/tonydle), Gabriel Novas (http://github.com/gabrinovas). MIT license, see LICENSE;
`ur_onrobot_control/launch_parts` derives from the BSD-licensed launch file of ur_robot_driver.

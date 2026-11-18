#!/usr/bin/env python3
# Copyright 2026 Tony Le, Gabriel Novas
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""
Jog the tool of the arm with MoveIt Servo and drive the gripper, from the keyboard.

  ros2 run ur_onrobot_control servo_keyboard_teleop.py --onrobot-type 2fg7

Keys, topics and speeds are in config/servo_keyboard_teleop.yaml.
"""

import argparse
import select
import sys
import termios
import tty

from control_msgs.action import GripperCommand
from geometry_msgs.msg import TwistStamped
from onrobot_description.config_loader import require
from onrobot_description.grippers import gripper_names, load_gripper
import onrobot_driver
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_srvs.srv import Trigger
from ur_onrobot_control.config import control_config


class ServoKeyboardTeleop(Node):

    def __init__(self, config, gripper):
        super().__init__('servo_keyboard_teleop')
        self.config = config
        depth = require(config, 'queue_depth')
        self.twist_publisher = self.create_publisher(
            TwistStamped, require(config, 'twist_topic'), depth)
        driver_arguments = require(onrobot_driver.driver_config(), 'launch_arguments')
        names = {
            'namespace': require(driver_arguments, 'ns', 'default'),
            'controller': require(
                onrobot_driver.load_driver_gripper(gripper), 'controllers', 'action')[0]}
        self.gripper_client = ActionClient(
            self, GripperCommand, require(config, 'gripper_action').format(**names))
        # Closed and open are the two ends of the first joint of the gripper (width or vacuum).
        description = load_gripper(gripper)
        joint = next(iter(require(description, 'joints')))
        limits = (require(description, 'joints', joint, 'lower'),
                  require(description, 'joints', joint, 'upper'))
        vacuum = require(description, 'kind') == 'vacuum'
        self.closed_position, self.open_position = (limits[1], limits[0]) if vacuum else limits
        self.linear_speed = require(config, 'linear_speed')
        self.angular_speed = require(config, 'angular_speed')
        self.frame_id = require(config, 'start_frame')

        start_servo = self.create_client(Trigger, require(config, 'start_servo_service'))
        if start_servo.wait_for_service(timeout_sec=require(config, 'service_wait_s')):
            start_servo.call_async(Trigger.Request())
            self.get_logger().info('MoveIt Servo started')
        else:
            self.get_logger().warn('The start service of MoveIt Servo is not available yet')

    def publish_twist(self, direction):
        """Publish the motion of one key: six values of -1, 0 or 1 times the speeds."""
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.frame_id
        linear, angular = message.twist.linear, message.twist.angular
        linear.x, linear.y, linear.z = (float(v * self.linear_speed) for v in direction[:3])
        angular.x, angular.y, angular.z = (float(v * self.angular_speed) for v in direction[3:])
        self.twist_publisher.publish(message)

    def move_gripper(self, close):
        wait_s = require(self.config, 'action_wait_s')
        if not self.gripper_client.wait_for_server(timeout_sec=wait_s):
            self.get_logger().warn('The gripper action is not available')
            return
        goal = GripperCommand.Goal()
        goal.command.position = float(self.closed_position if close else self.open_position)
        goal.command.max_effort = float(require(self.config, 'gripper_max_effort'))
        self.gripper_client.send_goal_async(goal)
        report(f"Gripper: {'closed / vacuum on' if close else 'open / vacuum off'}")

    def change_speed(self, sign):
        lowest, highest = require(self.config, 'linear_speed_range')
        step = sign * require(self.config, 'linear_speed_step')
        self.linear_speed = min(highest, max(lowest, self.linear_speed + step))
        report(f'Linear speed: {self.linear_speed:.2f} m/s')


def report(text):
    """Write a status line over the previous one."""
    print(f'\r  {text:<50}', end='')


def read_key(settings, wait_s):
    """Return the key pressed within the wait, or '' when there was none."""
    tty.setraw(sys.stdin.fileno())
    ready, _, _ = select.select([sys.stdin], [], [], wait_s)
    key = sys.stdin.read(1) if ready else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key.lower()


def main():
    parser = argparse.ArgumentParser(description='Keyboard jogging with MoveIt Servo')
    parser.add_argument('--onrobot-type', required=True, choices=gripper_names())
    arguments = parser.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    config = control_config('servo_keyboard_teleop')
    print(require(config, 'help'))
    if not sys.stdin.isatty():
        print('This program needs an interactive terminal (for example docker exec -it ...).')
        return 1

    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = ServoKeyboardTeleop(config, arguments.onrobot_type)
    still = [0, 0, 0, 0, 0, 0]
    try:
        while rclpy.ok():
            key = read_key(settings, require(config, 'key_wait_s'))
            direction = require(config, 'motion_keys').get(key, still)
            if key in require(config, 'frames'):
                node.frame_id = require(config, 'frames', key)
                report(f'Frame: {node.frame_id}')
            elif key in require(config, 'faster_keys'):
                node.change_speed(1)
            elif key in require(config, 'slower_keys'):
                node.change_speed(-1)
            elif key == require(config, 'close_key'):
                node.move_gripper(True)
            elif key == require(config, 'open_key'):
                node.move_gripper(False)
            elif key in require(config, 'quit_keys'):
                break
            node.publish_twist(direction)
            rclpy.spin_once(node, timeout_sec=0.0)
    finally:
        node.publish_twist(still)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())

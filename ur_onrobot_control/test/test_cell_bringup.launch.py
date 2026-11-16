# Copyright 2026 Tony Le, Gabriel Novas
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Start a simulated cell with the watchdog and check what it publishes."""

import time
import unittest

from diagnostic_msgs.msg import DiagnosticArray
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import launch_testing.actions
import launch_testing.markers
from onrobot_description.config_loader import package_file
import pytest
import rclpy
from sensor_msgs.msg import JointState

WAIT_S = 30.0


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            package_file('ur_onrobot_control', 'launch', 'ur_onrobot_bringup.launch.py')),
        launch_arguments={
            'use_fake_hardware': 'true',
            'ur_type': 'ur5e',
            'onrobot_type': '2fg7',
            'cell': 'basic',
            'launch_rviz': 'false',
            'launch_watchdog': 'true',
        }.items())
    return LaunchDescription([bringup, launch_testing.actions.ReadyToTest()])


class TestCellBringup(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_cell_bringup_checker')

    def tearDown(self):
        self.node.destroy_node()

    def collect(self, message_type, topic, done):
        """Spin until done(messages) is true or the wait ends; return the messages received."""
        messages = []
        subscription = self.node.create_subscription(message_type, topic, messages.append, 10)
        deadline = time.time() + WAIT_S
        while time.time() < deadline and not done(messages):
            rclpy.spin_once(self.node, timeout_sec=0.2)
        self.node.destroy_subscription(subscription)
        return messages

    def test_the_merged_joint_states_hold_the_arm_and_the_gripper(self):
        def complete(messages):
            expected = {'shoulder_pan_joint', 'finger_width'}
            return bool(messages) and expected <= set(messages[-1].name)

        messages = self.collect(JointState, '/merged_joint_states', complete)
        self.assertTrue(messages, 'No message on /merged_joint_states')
        self.assertTrue(complete(messages), f'Joints received: {messages[-1].name}')

    def test_the_watchdog_reports_a_normal_state(self):
        def watchdog_statuses(messages):
            return [status for array in messages for status in array.status
                    if 'Safety & Interlock Monitor' in status.name]

        messages = self.collect(DiagnosticArray, '/diagnostics', watchdog_statuses)
        statuses = watchdog_statuses(messages)
        self.assertTrue(statuses, 'The watchdog published no diagnostics')
        self.assertEqual(ord(statuses[-1].level), 0, statuses[-1].message)

# Copyright 2026 Tony Le, Gabriel Novas
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Check that the description is generated for every cell and gripper, from the command helper."""

import xml.etree.ElementTree as ET

from launch import LaunchContext
from launch.substitutions import Command
from onrobot_description.config_loader import require
from onrobot_description.grippers import gripper_names
import pytest
from ur_onrobot_description.description_command import description_name, robot_description_command
from ur_workspace_description.cells import cell_names, load_cell


def generate(cell, gripper, **overrides):
    """Return the root of the description of a cell with a gripper."""
    arguments = {
        'cell': cell, 'ur_type': 'ur5e', 'onrobot_type': gripper, 'robot_ip': '192.0.2.1',
        'use_fake_hardware': 'true', 'tf_prefix': '', 'include_gripper_ros2_control': 'true'}
    arguments.update(overrides)
    return ET.fromstring(Command(robot_description_command(**arguments)).perform(LaunchContext()))


@pytest.mark.parametrize('cell', cell_names())
@pytest.mark.parametrize('gripper', gripper_names())
def test_every_cell_and_gripper_gives_a_description(cell, gripper):
    root = generate(cell, gripper)
    assert root.attrib['name'] == description_name(cell)
    links = {link.attrib['name'] for link in root.findall('link')}
    assert {'world', 'tool0', 'onrobot_base_link', 'gripper_tcp'} <= links
    has_furniture = require(load_cell(cell), 'environment') is not None
    assert ('table_top' in links) == has_furniture
    assert {block.attrib['name'] for block in root.findall('ros2_control')} == {
        'ur5e', 'OnRobotGripper'}


def test_the_gripper_hardware_can_be_left_out():
    root = generate('right', '2fg7', include_gripper_ros2_control='false')
    assert [block.attrib['name'] for block in root.findall('ros2_control')] == ['ur5e']


def test_the_prefix_reaches_the_arm_and_the_gripper():
    root = generate('basic', '3fg15', tf_prefix='cell_')
    joints = {joint.attrib['name'] for joint in root.findall('joint')}
    expected = {'cell_shoulder_pan_joint', 'cell_finger_width', 'cell_onrobot_base_link_joint'}
    assert expected <= joints


def test_the_robot_base_is_turned_as_the_cell_says():
    for cell in cell_names():
        root = generate(cell, '2fg7')
        joints = {joint.attrib['name']: joint for joint in root.findall('joint')}
        base = joints['base_joint']
        yaw = float(base.find('origin').attrib['rpy'].split()[2])
        assert yaw == pytest.approx(require(load_cell(cell), 'robot', 'base_yaw'))

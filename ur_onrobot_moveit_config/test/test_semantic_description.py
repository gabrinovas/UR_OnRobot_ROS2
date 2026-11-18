# Copyright 2026 Tony Le, Gabriel Novas
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Check that the semantic description is generated for every cell and gripper."""

import xml.etree.ElementTree as ET

from launch import LaunchContext
from onrobot_description.config_loader import require
from onrobot_description.grippers import gripper_names
import pytest
from ur_onrobot_description.description_command import description_name
from ur_onrobot_moveit_config.parameters import load_file, moveit_config, semantic_description
from ur_workspace_description.cells import cell_names, load_cell


def generate(cell, gripper, tf_prefix=''):
    values = {'cell': cell, 'onrobot_type': gripper, 'tf_prefix': tf_prefix}
    parameter = semantic_description(moveit_config(), values)['robot_description_semantic']
    return ET.fromstring(parameter.value[0].perform(LaunchContext()))


@pytest.mark.parametrize('cell', cell_names())
@pytest.mark.parametrize('gripper', gripper_names())
def test_every_cell_and_gripper_gives_a_semantic_description(cell, gripper):
    root = generate(cell, gripper)
    assert root.attrib['name'] == description_name(cell)
    groups = {group.attrib['name'] for group in root.findall('group')}
    assert groups >= {'ur_manipulator', 'gripper'}
    states = {state.attrib['name']: state for state in root.findall('group_state')}
    pan = float(states['home'].find('joint').attrib['value'])
    assert pan == pytest.approx(require(load_cell(cell), 'moveit', 'home_shoulder_pan'))
    pairs = root.findall('disable_collisions')
    furniture = any(pair.attrib['link2'] == 'table_top' for pair in pairs)
    assert furniture == (require(load_cell(cell), 'environment') is not None)


def test_the_prefix_reaches_the_planning_group():
    root = generate('basic', '2fg7', tf_prefix='cell_')
    chain = root.find('group').find('chain')
    assert chain.attrib == {'base_link': 'cell_base_link', 'tip_link': 'cell_gripper_tcp'}


def test_the_files_named_in_the_configuration_exist():
    config = moveit_config()
    for key in ('kinematics', 'joint_limits', 'ompl', 'pilz', 'moveit_controllers', 'servo'):
        assert load_file(config, key)
    controllers = load_file(config, 'moveit_controllers')
    section = controllers[require(config, 'controller_manager_section')]
    assert set(require(config, 'arm_controllers')) <= set(section)

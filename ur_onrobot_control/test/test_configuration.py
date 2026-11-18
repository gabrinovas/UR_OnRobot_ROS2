# Copyright 2026 Tony Le, Gabriel Novas
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Check the configuration files of the control package against what reads them."""

from onrobot_description.config_loader import package_file, require
from onrobot_description.grippers import gripper_names
from onrobot_description.launch_arguments import declare_arguments
import pytest
from ur_onrobot_control.config import control_config, PACKAGE, ur_types
from ur_onrobot_control.launch_parts.spawners import controller_lists
from ur_workspace_description.cells import cell_names
import yaml

CONFIGURATIONS = {
    'bringup': ['launch_arguments', 'selector_launch_arguments'],
    'ur_control': ['launch_arguments'],
    'safety_watchdog': ['launch_arguments'],
    'foxglove': ['launch_arguments'],
}


@pytest.mark.parametrize('name', CONFIGURATIONS)
def test_every_argument_table_can_be_declared(name):
    config = control_config(name)
    for table in CONFIGURATIONS[name]:
        assert declare_arguments(config, table)


def test_the_supported_robots_are_the_update_rate_files():
    assert 'ur5e' in ur_types()
    assert len(ur_types()) == 14


def test_the_parameter_files_of_the_control_node_exist():
    files = require(control_config('ur_control'), 'parameter_files')
    for ur_type in ur_types():
        package_file(PACKAGE, files['update_rate'].format(ur_type=ur_type))
    for path in files['controllers']:
        package_file(PACKAGE, path)


def test_every_controller_named_in_the_lists_is_known_to_the_manager():
    config = control_config('ur_control')
    manager_file = package_file(PACKAGE, require(config, 'parameter_files')['controllers'][0])
    with open(manager_file) as stream:
        known = set(yaml.safe_load(stream)['controller_manager']['ros__parameters'])
    lists = require(config, 'controllers')
    named = set(lists['active']) | set(lists['inactive']) | set(lists['consistent'])
    assert named <= known, f'unknown controllers: {named - known}'


def values(**overrides):
    base = {'load_freedrive_controller': 'false', 'activate_joint_controller': 'true',
            'initial_joint_controller': 'scaled_joint_trajectory_controller',
            'use_fake_hardware': 'true'}
    base.update(overrides)
    return base


def test_the_freedrive_controller_is_loaded_only_on_request():
    config = control_config('ur_control')
    freedrive = require(config, 'controllers', 'freedrive')
    assert freedrive not in sum(controller_lists(config, values()), [])
    assert freedrive in controller_lists(config, values(load_freedrive_controller='true'))[1]
    assert freedrive in controller_lists(config, values(initial_joint_controller=freedrive))[0]


def test_the_initial_controller_is_active_and_the_real_robot_ones_are_left_out_in_simulation():
    config = control_config('ur_control')
    active, inactive = controller_lists(config, values())
    assert 'scaled_joint_trajectory_controller' in active
    assert 'scaled_joint_trajectory_controller' not in inactive
    assert 'tcp_pose_broadcaster' not in active
    assert 'tcp_pose_broadcaster' in controller_lists(config, values(use_fake_hardware='false'))[0]


def test_the_watchdog_knows_how_to_hold_every_kind_of_gripper():
    from onrobot_description.grippers import load_gripper
    holds = require(control_config('safety_watchdog'), 'holds')
    for gripper in gripper_names():
        kind = require(load_gripper(gripper), 'kind')
        assert {'hold_current_position', 'hold_position', 'hold_max_effort'} <= set(holds[kind])


def test_the_cells_offered_are_the_cell_files():
    assert cell_names() == ['basic', 'left', 'right']


def test_the_modes_fix_neither_the_watchdog_nor_the_address_of_the_robot():
    modes = require(control_config('bringup'), 'modes')
    for values in modes.values():
        assert 'launch_watchdog' not in values and 'robot_ip' not in values
    assert require(control_config('bringup'), 'ur_driver', 'headless_mode') is False

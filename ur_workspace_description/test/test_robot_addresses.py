# Copyright 2026 Tony Le, Gabriel Novas
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Check the reader of the robot addresses file and the choice of the address of a launch."""

from onrobot_description.config_loader import ConfigError, package_file, require
import pytest
from ur_workspace_description.cells import cell_names
from ur_workspace_description.robot_addresses import addresses_path, calibration_file
from ur_workspace_description.robot_addresses import cell_addresses, compute_box_address
from ur_workspace_description.robot_addresses import default_address
from ur_workspace_description.robot_addresses import load_addresses, settings
from ur_workspace_description.robot_choice import choose_address, resolve_robot

ADDRESSES = """
calibration_directory: {directory}
addresses:
  192.0.2.101: {{cells: [right, basic], kinematics_calibration: null}}
  192.0.2.105: {{cells: [left, basic], kinematics_calibration: ur5e/192.0.2.105.yaml}}
  192.0.2.194: {{cells: [another_robot], kinematics_calibration: null}}
default: {{basic: 192.0.2.101, left: 192.0.2.105, right: 192.0.2.101, another_robot: 192.0.2.194}}
gripper_compute_box: {{basic: 192.0.2.1, left: 192.0.2.2}}
"""


@pytest.fixture
def addresses(tmp_path, monkeypatch):
    """Point the environment variable to a file of a temporary installation and return it."""
    (tmp_path / 'ur5e').mkdir()
    (tmp_path / 'ur5e' / '192.0.2.105.yaml').write_text('kinematics: {}\n')
    path = tmp_path / 'robot_addresses.yaml'
    path.write_text(ADDRESSES.format(directory=tmp_path))
    monkeypatch.setenv(require(settings(), 'environment_variable'), str(path))
    return path


def answering(*alive):
    return lambda address: address in alive


def test_the_template_is_a_valid_file_that_covers_every_cell():
    template = package_file('ur_workspace_description', require(settings(), 'template'))
    content = load_addresses(template)
    for cell in cell_names():
        assert default_address(content, cell) in cell_addresses(content, cell)


def test_the_addresses_of_a_cell_and_its_default(addresses):
    content = load_addresses()
    assert content['__file__'] == addresses_path() == str(addresses)
    assert cell_addresses(content, 'basic') == ['192.0.2.101', '192.0.2.105']
    assert cell_addresses(content, 'right') == ['192.0.2.101']
    assert default_address(content, 'basic') == '192.0.2.101'
    with pytest.raises(ConfigError, match=r"\[Fail-Fast\].*no address for cell 'unknown'"):
        default_address(content, 'unknown')


def test_without_the_file_the_error_names_the_variable_and_the_template(monkeypatch, tmp_path):
    variable = require(settings(), 'environment_variable')
    monkeypatch.delenv(variable, raising=False)
    message = rf'\[Fail-Fast\].*robot_addresses.template.yaml.*{variable}'
    with pytest.raises(ConfigError, match=message):
        addresses_path()
    monkeypatch.setenv(variable, str(tmp_path / 'missing.yaml'))
    with pytest.raises(ConfigError, match=r'\[Fail-Fast\].*missing.yaml'):
        load_addresses()


@pytest.mark.parametrize('good, broken', [
    ('default: {basic: 192.0.2.101', 'default: {basic: 192.0.2.9'),        # default not listed
    ('default: {basic: 192.0.2.101', 'default: {basic: 192.0.2.194'),     # default of another cell
    ('cells: [left, basic]', 'cells: left'),                               # cells is not a list
    (', kinematics_calibration: null}\n  192.0.2.105', '}\n  192.0.2.105'),  # calibration missing
    ('gripper_compute_box: {basic: 192.0.2.1, left: 192.0.2.2}', ''),      # section missing
    ('gripper_compute_box: {basic: 192.0.2.1, left: 192.0.2.2}', 'gripper_compute_box: 192.0.2.1'),
])
def test_an_incoherent_file_is_rejected(tmp_path, good, broken):
    valid = ADDRESSES.format(directory=tmp_path)
    assert good in valid
    text = valid.replace(good, broken)
    path = tmp_path / 'robot_addresses.yaml'
    path.write_text(text)
    with pytest.raises(ConfigError, match=r'\[Fail-Fast\]'):
        load_addresses(str(path))


def test_the_address_that_answers_is_chosen_else_the_default(addresses):
    content = load_addresses()
    assert choose_address(content, 'basic', answering('192.0.2.105'))[0] == '192.0.2.105'
    assert choose_address(content, 'basic', answering('192.0.2.101'))[0] == '192.0.2.101'
    address, reason = choose_address(content, 'basic', answering())
    assert address == '192.0.2.101' and 'none answers' in reason
    address, reason = choose_address(content, 'basic', answering('192.0.2.101', '192.0.2.105'))
    assert address == '192.0.2.101' and '192.0.2.101, 192.0.2.105 answer' in reason


def test_a_cell_with_one_address_is_not_asked_on_the_network(addresses):
    def never(address):
        raise AssertionError(f'{address} was asked')
    assert choose_address(load_addresses(), 'right', never)[0] == '192.0.2.101'


def test_the_compute_box_of_a_cell(addresses):
    content = load_addresses()
    assert compute_box_address(content, 'left') == '192.0.2.2'
    assert compute_box_address(content, 'right') is None


def test_the_calibration_of_an_address(addresses, tmp_path):
    content = load_addresses()
    assert calibration_file(content, '192.0.2.101') is None
    assert calibration_file(content, '198.51.100.7') is None
    assert calibration_file(content, '192.0.2.105') == str(tmp_path / 'ur5e' / '192.0.2.105.yaml')
    (tmp_path / 'ur5e' / '192.0.2.105.yaml').unlink()
    with pytest.raises(ConfigError, match=r'\[Fail-Fast\].*192.0.2.105.*does not exist'):
        calibration_file(content, '192.0.2.105')


def test_the_simulation_needs_no_file(monkeypatch):
    monkeypatch.delenv(require(settings(), 'environment_variable'), raising=False)
    choice = resolve_robot('basic', '192.0.2.105', True)
    assert choice == (str(require(settings(), 'simulation_address')), '', '', [], [])


def test_the_real_robot_gets_its_address_and_its_calibration(addresses, tmp_path):
    choice = resolve_robot('basic', '', False, answering('192.0.2.105'))
    assert choice.address == '192.0.2.105'
    assert choice.calibration == str(tmp_path / 'ur5e' / '192.0.2.105.yaml')
    assert choice.compute_box == '192.0.2.1'
    assert not choice.warnings and len(choice.notes) == 2

    choice = resolve_robot('right', '', False, answering())
    assert (choice.address, choice.calibration, choice.compute_box) == ('192.0.2.101', '', '')
    assert any('no kinematics calibration' in text for text in choice.warnings)


def test_an_address_given_to_the_launch_wins_and_is_checked_against_the_cell(addresses):
    choice = resolve_robot('right', '192.0.2.105', False, answering('192.0.2.101'))
    assert choice.address == '192.0.2.105' and choice.calibration
    assert "not listed for cell 'right'" in choice.warnings[0]
    assert not resolve_robot('left', '192.0.2.105', False).warnings


def test_a_cell_without_compute_box_is_warned_about(addresses):
    warnings = resolve_robot('right', '', False, answering()).warnings
    assert any('no gripper Compute Box' in text for text in warnings)


def test_the_real_robot_without_the_file_fails(monkeypatch):
    monkeypatch.delenv(require(settings(), 'environment_variable'), raising=False)
    with pytest.raises(ConfigError, match=r'\[Fail-Fast\]'):
        resolve_robot('basic', '', False, answering())

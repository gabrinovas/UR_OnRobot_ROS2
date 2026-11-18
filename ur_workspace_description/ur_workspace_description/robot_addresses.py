# Copyright 2026 Tony Le, Gabriel Novas
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""
The private file with the addresses of the robots of an installation.

It says which cells every address can be used in, the default address of each cell, the
kinematics calibration extracted from each robot and the Compute Box of the gripper of each
cell. It is not part of the repository: config/robot_addresses.template.yaml shows its format
and where to put it.
"""

import os

from ament_index_python.packages import get_package_share_directory
from onrobot_description.config_loader import ConfigError, load_config, package_file, require

PACKAGE = 'ur_workspace_description'


def settings():
    """Return the section of the package configuration about the addresses file."""
    config = load_config(package_file(PACKAGE, 'config', 'ur_workspace_description.yaml'))
    return dict(require(config, 'robot_addresses'), __file__=config['__file__'])


def addresses_path():
    """Return the path of the addresses file: the environment variable, else the package copy."""
    found = settings()
    variable = require(found, 'environment_variable')
    if os.environ.get(variable):
        return os.path.expanduser(os.environ[variable])
    share = get_package_share_directory(PACKAGE)
    path = os.path.join(share, require(found, 'package_file'))
    if os.path.isfile(path):
        return path
    raise ConfigError(
        f'[Fail-Fast] The addresses of the robots are not known. Copy '
        f"'{os.path.join(share, require(found, 'template'))}', fill it in and set the "
        f"environment variable {variable} to its path (or install it as '{path}')")


def load_addresses(path=None):
    """Load the addresses file and check that every part of it is coherent."""
    content = load_config(path or addresses_path())
    source = content['__file__']
    require(content, 'calibration_directory')
    entries = require(content, 'addresses')
    if not isinstance(entries, dict) or not entries:
        raise ConfigError(f"[Fail-Fast] 'addresses' of '{source}' must list at least one address")
    for address in entries:
        if not isinstance(require(content, 'addresses', address, 'cells'), list):
            raise ConfigError(
                f"[Fail-Fast] 'addresses.{address}.cells' of '{source}' must be a list")
        require(content, 'addresses', address, 'kinematics_calibration')
    for cell, address in require(content, 'default').items():
        if cell not in entries.get(address, {}).get('cells', []):
            raise ConfigError(
                f"[Fail-Fast] 'default.{cell}' of '{source}' is {address}, which is not an "
                f"address listed for cell '{cell}'")
    if not isinstance(require(content, 'gripper_compute_box'), dict):
        raise ConfigError(
            f"[Fail-Fast] 'gripper_compute_box' of '{source}' must map cells to addresses "
            '({} when no gripper has a Compute Box)')
    return content


def cell_addresses(content, cell):
    """Return the addresses that can be used in a cell, in the order of the file."""
    return [address for address, entry in require(content, 'addresses').items()
            if cell in entry['cells']]


def default_address(content, cell):
    """Return the default address of a cell; a cell without one is an error."""
    defaults = require(content, 'default')
    if cell not in defaults:
        raise ConfigError(
            f"[Fail-Fast] '{content['__file__']}' has no address for cell '{cell}': add the cell "
            "to the 'cells' of an address and to 'default'")
    return defaults[cell]


def compute_box_address(content, cell):
    """Return the address of the Compute Box of the gripper of a cell, or None when it has none."""
    address = require(content, 'gripper_compute_box').get(cell)
    return None if address is None else str(address)


def calibration_file(content, address):
    """Return the kinematics calibration file of an address, or None when it has none."""
    entry = require(content, 'addresses').get(address)
    if entry is None or entry['kinematics_calibration'] is None:
        return None
    directory = os.path.expanduser(str(require(content, 'calibration_directory')))
    path = os.path.join(directory, str(entry['kinematics_calibration']))
    if not os.path.isfile(path):
        raise ConfigError(
            f"[Fail-Fast] The kinematics calibration of {address} named in "
            f"'{content['__file__']}' does not exist: '{path}'")
    return path

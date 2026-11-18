# Copyright 2026 Tony Le, Gabriel Novas
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""The cells known to this workspace: one YAML file per cell in config/cells."""

import os

from ament_index_python.packages import get_package_share_directory
from onrobot_description.config_loader import ConfigError, load_config, require

PACKAGE = 'ur_workspace_description'
CELLS_DIRECTORY = os.path.join('config', 'cells')


def cells_directory():
    """Return the installed directory that holds one file per cell."""
    return os.path.join(get_package_share_directory(PACKAGE), CELLS_DIRECTORY)


def cell_names(with_environment=False):
    """Return the names of the cells, sorted; with_environment keeps those that have furniture."""
    files = [name for name in os.listdir(cells_directory()) if name.endswith('.yaml')]
    names = sorted(name[:-len('.yaml')] for name in files)
    if with_environment:
        names = [name for name in names if require(load_cell(name), 'environment') is not None]
    if not names:
        raise ConfigError(f"[Fail-Fast] No cell file found in '{cells_directory()}'")
    return names


def load_cell(name):
    """Load one cell; an unknown name is an error that lists the valid ones."""
    path = os.path.join(cells_directory(), f'{name}.yaml')
    if not os.path.isfile(path):
        files = [entry for entry in os.listdir(cells_directory()) if entry.endswith('.yaml')]
        available = ', '.join(sorted(entry[:-len('.yaml')] for entry in files))
        raise ConfigError(f"[Fail-Fast] Unknown cell '{name}'. Available cells: {available}")
    return load_config(path)

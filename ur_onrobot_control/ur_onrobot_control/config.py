# Copyright 2026 Tony Le, Gabriel Novas
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Configuration files of this package and small helpers to read launch values."""

import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from onrobot_description.config_loader import load_config, package_file, require

PACKAGE = 'ur_onrobot_control'


def control_config(name):
    """Return the content of config/<name>.yaml of this package."""
    return load_config(package_file(PACKAGE, 'config', f'{name}.yaml'))


def section(config, *keys):
    """Return a section of a configuration that still knows the file it came from."""
    return dict(require(config, *keys), __file__=config['__file__'])


def launch_values(context, config, table='launch_arguments'):
    """Return the value of every argument of a table, as text."""
    return {name: LaunchConfiguration(name).perform(context) for name in require(config, table)}


def is_true(text):
    """Return whether a launch value means yes."""
    return str(text).lower() == 'true'


def ur_types():
    """Return the supported models of robot: the files of the update rate directory."""
    directory = os.path.join(
        get_package_share_directory(PACKAGE),
        require(control_config('ur_control'), 'update_rate_directory'))
    return sorted(name[:-len('.yaml')] for name in os.listdir(directory) if name.endswith('.yaml'))


def include(package, launch_file, arguments, **options):
    """Return the inclusion of a launch file of a package with the given arguments (as text)."""
    source = PythonLaunchDescriptionSource(package_file(package, 'launch', launch_file))
    items = [(name, str(value)) for name, value in arguments.items()]
    return IncludeLaunchDescription(source, launch_arguments=items, **options)

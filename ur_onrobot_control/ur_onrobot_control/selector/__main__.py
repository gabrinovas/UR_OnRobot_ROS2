# Copyright 2026 Tony Le, Gabriel Novas
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Command line of the selector: python3 -m ur_onrobot_control.selector."""

import argparse
import json
import sys

from onrobot_description.config_loader import ConfigError
from onrobot_description.grippers import gripper_names
from ur_onrobot_control.selector import RESULT_KEYS
from ur_workspace_description.cells import cell_names
from ur_workspace_description.robot_addresses import cell_addresses, load_addresses


def main():
    parser = argparse.ArgumentParser(description='Choose how to launch a UR + OnRobot cell')
    for key in RESULT_KEYS:
        parser.add_argument(f'--{key}', required=True, help='value selected at start')
    parser.add_argument('--output', help='file the choices are written to, as JSON')
    arguments = parser.parse_args()

    from ur_onrobot_control.selector.app import CellSelector
    options = {'cells': {cell: [] for cell in cell_names()}, 'grippers': gripper_names(),
               'addresses_problem': ''}
    try:  # the simulation can be launched without the addresses of the robots
        content = load_addresses()
        options['cells'] = {cell: cell_addresses(content, cell) for cell in cell_names()}
    except ConfigError as problem:
        options['addresses_problem'] = str(problem)
    result = CellSelector(options, {key: getattr(arguments, key) for key in RESULT_KEYS}).run()
    if result is None:
        return 1
    if arguments.output:
        with open(arguments.output, 'w') as stream:
            json.dump(result, stream)
    else:
        print(json.dumps(result))
    return 0


if __name__ == '__main__':
    sys.exit(main())

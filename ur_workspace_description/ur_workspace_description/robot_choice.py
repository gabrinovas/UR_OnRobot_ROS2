# Copyright 2026 Tony Le, Gabriel Novas
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Choose the address and the kinematics calibration of the robot a launch file talks to."""

from collections import namedtuple
from concurrent.futures import ThreadPoolExecutor
import subprocess

from onrobot_description.config_loader import require
from ur_workspace_description.robot_addresses import calibration_file, cell_addresses
from ur_workspace_description.robot_addresses import compute_box_address, default_address
from ur_workspace_description.robot_addresses import load_addresses, settings

# address: text; calibration: path of the kinematics file, '' for the generic kinematics;
# compute_box: address of the Compute Box of the gripper of the cell, '' when it has none;
# notes and warnings: what the launch file has to tell the user about the choice.
RobotChoice = namedtuple('RobotChoice', 'address calibration compute_box notes warnings')


def ping(address):
    """Return whether an address answers on the network within the configured wait."""
    wait = require(settings(), 'ping')
    command = ['ping', '-c', str(require(wait, 'count')), '-W', str(require(wait, 'wait_s')),
               str(address)]
    try:
        finished = subprocess.run(command, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except OSError:
        return False
    return finished.returncode == 0


def choose_address(content, cell, answers=ping):
    """
    Return (address, reason) for a cell when the user gave no address.

    The address of the cell that answers on the network is used when exactly one does; with none
    or several, the default of the cell. answers(address) -> bool tests one address.
    """
    allowed = cell_addresses(content, cell)
    default = default_address(content, cell)
    if len(allowed) == 1:
        return default, f"the only address of cell '{cell}'"
    with ThreadPoolExecutor(max_workers=len(allowed)) as pool:
        alive = [address for address, up in zip(allowed, pool.map(answers, allowed)) if up]
    if len(alive) == 1:
        return alive[0], f"the only address of cell '{cell}' that answers ({', '.join(allowed)})"
    found = 'none answers' if not alive else f"{', '.join(alive)} answer"
    return default, f"the default of cell '{cell}' ({found} of {', '.join(allowed)})"


def resolve_robot(cell, robot_ip, simulated, answers=ping):
    """
    Return the RobotChoice of a launch: which robot it talks to and with which kinematics.

    simulated: the address written in the description is the one of the configuration and the
    addresses file is not read. Real robot: robot_ip when given, else the choice among the
    addresses of the cell; the calibration is the one the file names for that address, and the
    Compute Box the one of the cell.
    """
    if simulated:
        return RobotChoice(str(require(settings(), 'simulation_address')), '', '', [], [])
    content = load_addresses()
    notes, warnings = [], []
    if robot_ip:
        address = robot_ip
        notes.append(f'Robot address {address}: given to the launch')
        if address not in cell_addresses(content, cell):
            warnings.append(
                f"Robot address {address} is not listed for cell '{cell}' in "
                f"'{content['__file__']}'")
    else:
        address, reason = choose_address(content, cell, answers)
        notes.append(f'Robot address {address}: {reason}')
    calibration = calibration_file(content, address)
    if calibration:
        notes.append(f'Kinematics calibration of the robot: {calibration}')
    else:
        warnings.append(
            f'Robot {address} has no kinematics calibration: the generic kinematics of the model '
            'are used, so the tool pose ROS computes can be a few millimetres off')
    compute_box = compute_box_address(content, cell) or ''
    if not compute_box:
        warnings.append(
            f"Cell '{cell}' has no gripper Compute Box in '{content['__file__']}': a gripper "
            'connected over tcp will not start')
    return RobotChoice(address, calibration or '', compute_box, notes, warnings)


def report(choice, logger):
    """Write the notes and the warnings of a choice to a logger."""
    for text in choice.notes:
        logger.info(text)
    for text in choice.warnings:
        logger.warning(text)

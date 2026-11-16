# Copyright 2026 Tony Le, Gabriel Novas
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""
Ask in a terminal window how to launch a cell.

The window runs in a process of its own because the launch system already runs an event loop.
"""

import json
import subprocess
import sys
import tempfile

RESULT_KEYS = ('use_fake_hardware', 'cell', 'onrobot_type', 'launch_rviz')


def run_selector(chosen):
    """
    Show the selector with `chosen` preselected and return the new choices.

    chosen and the result are dicts with the keys RESULT_KEYS. Returns None when it is cancelled.
    """
    with tempfile.NamedTemporaryFile('r', suffix='.json') as output:
        command = [sys.executable, '-m', 'ur_onrobot_control.selector', '--output', output.name]
        for key in RESULT_KEYS:
            command += [f'--{key}', str(chosen[key])]
        if subprocess.run(command).returncode != 0:
            return None
        return json.load(output)

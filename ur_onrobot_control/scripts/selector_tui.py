#!/usr/bin/env python3
# Copyright 2026 Tony Le, Gabriel Novas
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Show the selector on its own and print the choices (ros2 run ur_onrobot_control ...)."""

import sys

from ur_onrobot_control.selector.__main__ import main

if __name__ == '__main__':
    sys.exit(main())

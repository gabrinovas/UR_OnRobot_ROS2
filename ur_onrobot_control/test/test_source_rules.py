# Copyright 2026 Tony Le, Gabriel Novas
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Check that no source file of this package is too long or holds non-ASCII characters."""

import os

from onrobot_description.source_rules import violations


def test_source_files_follow_the_rules():
    package_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
    assert violations(package_root) == []

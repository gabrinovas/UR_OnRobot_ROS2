# Copyright 2026 Tony Le, Gabriel Novas
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Read the parameters of a node that has no value of its own: every one must be given."""

from rclpy.parameter import Parameter


def required(node, name, kind):
    """
    Return the value of a parameter that must be set.

    kind is one of the rclpy Parameter.Type values. A parameter that was not given stops the node
    with a [Fail-Fast] error naming it.
    """
    if not node.has_parameter(name):
        node.declare_parameter(name, kind)
    value = node.get_parameter(name).value
    if value is None:
        owner = node.get_fully_qualified_name()
        raise RuntimeError(f"[Fail-Fast] Parameter '{name}' of node '{owner}' is not set")
    return value


STRING = Parameter.Type.STRING
STRING_ARRAY = Parameter.Type.STRING_ARRAY
INTEGER = Parameter.Type.INTEGER
DOUBLE = Parameter.Type.DOUBLE
BOOL = Parameter.Type.BOOL

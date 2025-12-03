#!/usr/bin/env python3

# Copyright (c) 2021 PickNik, Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the {copyright_holder} nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
Common utilities for loading YAML configuration files in ROS2 MoveIt2 launch files.
"""

import math
import os
import yaml

from ament_index_python.packages import get_package_share_directory


def construct_angle_radians(loader, node):
    """Utility function to construct radian values from yaml."""
    value = loader.construct_scalar(node)
    try:
        return float(value)
    except (SyntaxError, ValueError) as e:
        raise yaml.YAMLError(f"Invalid expression for radians: {value}") from e


def construct_angle_degrees(loader, node):
    """Utility function for converting degrees into radians from yaml."""
    return math.radians(construct_angle_radians(loader, node))


def load_yaml(package_name, file_path):
    """
    Load a YAML file from a ROS2 package.
    
    Args:
        package_name (str): Name of the ROS2 package
        file_path (str): Relative path to the YAML file within the package
        
    Returns:
        dict: Parsed YAML content, or None if file not found
        
    Raises:
        FileNotFoundError: If the package or file doesn't exist
        yaml.YAMLError: If the YAML is malformed
    """
    try:
        package_path = get_package_share_directory(package_name)
    except Exception as e:
        raise FileNotFoundError(f"Package '{package_name}' not found") from e
    
    absolute_file_path = os.path.join(package_path, file_path)
    
    return load_yaml_abs(absolute_file_path)


def load_yaml_abs(absolute_file_path):
    """
    Load a YAML file from an absolute path.
    
    Args:
        absolute_file_path (str): Absolute path to the YAML file
        
    Returns:
        dict: Parsed YAML content, or None if file not found
        
    Raises:
        FileNotFoundError: If the file doesn't exist
        yaml.YAMLError: If the YAML is malformed
    """
    # Register custom YAML constructors for angle handling
    try:
        yaml.SafeLoader.add_constructor("!radians", construct_angle_radians)
        yaml.SafeLoader.add_constructor("!degrees", construct_angle_degrees)
    except Exception as e:
        raise RuntimeError("YAML support not available; install python-yaml") from e
    
    if not os.path.isfile(absolute_file_path):
        raise FileNotFoundError(f"YAML file not found: {absolute_file_path}")
    
    try:
        with open(absolute_file_path, 'r', encoding='utf-8') as file:
            content = yaml.safe_load(file)
            if content is None:
                # Empty YAML file
                return {}
            return content
    except yaml.YAMLError as e:
        raise yaml.YAMLError(f"Error parsing YAML file {absolute_file_path}: {e}") from e
    except OSError as e:
        raise OSError(f"Error reading file {absolute_file_path}: {e}") from e


def load_yaml_with_substitutions(package_name, file_path, context):
    """
    Load YAML with launch substitutions resolved.
    
    Args:
        package_name (str): Name of the ROS2 package
        file_path (str): Relative path to the YAML file
        context: Launch context for substitution
        
    Returns:
        dict: Parsed YAML with substitutions
    """
    # If file_path contains substitutions, resolve them
    if hasattr(file_path, 'perform'):
        file_path = file_path.perform(context)
    
    return load_yaml(package_name, file_path)


def merge_yaml(base_dict, override_dict):
    """
    Recursively merge two YAML dictionaries.
    
    Args:
        base_dict (dict): Base dictionary
        override_dict (dict): Dictionary with override values
        
    Returns:
        dict: Merged dictionary
    """
    if not isinstance(base_dict, dict) or not isinstance(override_dict, dict):
        return override_dict
    
    result = base_dict.copy()
    for key, value in override_dict.items():
        if key in result and isinstance(result[key], dict) and isinstance(value, dict):
            result[key] = merge_yaml(result[key], value)
        else:
            result[key] = value
    
    return result


# Para compatibilidad con versiones anteriores
load_yaml_file = load_yaml

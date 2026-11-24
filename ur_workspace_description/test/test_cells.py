# Copyright 2026 Tony Le, Gabriel Novas
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Check the cell files and the descriptions generated from them."""

import os
import subprocess
import xml.etree.ElementTree as ET

from onrobot_description.config_loader import ConfigError, package_file, require
import pytest
from ur_workspace_description.cells import cell_names, load_cell


def test_the_three_cells_are_listed():
    assert cell_names() == ['basic', 'left', 'right']
    assert cell_names(with_environment=True) == ['left', 'right']


@pytest.mark.parametrize('cell', cell_names())
def test_every_cell_file_has_what_its_readers_need(cell):
    content = load_cell(cell)
    require(content, 'description_name')
    for key in ('base_yaw', 'cable_connector'):
        require(content, 'robot', key)
    require(content, 'robot', 'script', 'package')
    require(content, 'robot', 'script', 'file')
    require(content, 'moveit', 'collision_exclusions')
    environment = require(content, 'environment')
    if environment is not None:
        package_file('ur_workspace_description', environment['file'])


def test_no_cell_file_holds_the_address_of_a_robot():
    for cell in cell_names():
        assert 'ip' not in require(load_cell(cell), 'robot')


def test_an_unknown_cell_names_the_valid_ones():
    with pytest.raises(ConfigError, match=r'\[Fail-Fast\].*basic, left, right'):
        load_cell('no_such_cell')


# What every piece of furniture collides with: meshes of meshes/environment.
COLLISION_MESHES = {
    'table_top': ['table.stl'],
    'table_top_2': ['table.stl'],
    'conveyor_belt': ['conveyor_body.stl', 'conveyor_rail_1.stl', 'conveyor_rail_2.stl'],
}


def furniture_links(cell):
    """Return the links, by name, of the description of the furniture of a cell."""
    description = package_file('ur_workspace_description', 'urdf', 'cell.urdf.xacro')
    result = subprocess.run(['xacro', description, f'cell:={cell}'],
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    assert result.returncode == 0, result.stderr
    return {link.attrib['name']: link for link in ET.fromstring(result.stdout).findall('link')}


@pytest.mark.parametrize('cell', cell_names(with_environment=True))
def test_the_furniture_of_a_cell_is_two_tables_and_a_conveyor(cell):
    # The safety barriers are legacy: commented out, neither shown nor used for collisions.
    assert set(furniture_links(cell)) == {'world', *COLLISION_MESHES}


@pytest.mark.parametrize('cell', cell_names(with_environment=True))
def test_every_piece_of_furniture_collides_where_it_is_seen(cell):
    links = furniture_links(cell)
    for name, meshes in COLLISION_MESHES.items():
        shown = links[name].find('visual')
        collisions = links[name].findall('collision')
        files = [collision.find('geometry/mesh').attrib['filename'] for collision in collisions]
        assert [os.path.basename(file) for file in files] == meshes
        for collision in collisions:
            assert collision.find('origin').attrib == shown.find('origin').attrib
            scale = collision.find('geometry/mesh').attrib['scale']
            assert scale == shown.find('geometry/mesh').attrib['scale']
        for mesh in meshes:
            package_file('ur_workspace_description', 'meshes', 'environment', mesh)


def test_the_cell_is_required_and_must_have_furniture():
    description = package_file('ur_workspace_description', 'urdf', 'cell.urdf.xacro')
    for arguments in ([], ['cell:=basic'], ['cell:=no_such_cell']):
        result = subprocess.run(['xacro', description, *arguments],
                                stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        assert result.returncode != 0, f'xacro accepted {arguments}'

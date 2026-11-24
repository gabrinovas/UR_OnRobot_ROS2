#!/usr/bin/env python3
# Copyright 2026 Tony Le, Gabriel Novas
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""
Make, from the conveyor measured in the lab, the meshes the descriptions load.

One mesh to look at, simplified, and three convex pieces to collide with: the conveyor up to
its belt and each of its two rails. Collision checkers fill a mesh up to its convex hull, so
one piece would close the gap between the rails, where the gripper works. The values are in
conveyor_meshes.yaml; a result outside its limits is an error and nothing is written.

    python3 meshes_original/conveyor_meshes.py --out meshes/environment

It needs numpy, scipy, yaml and vtk.
"""

import argparse
import os

import mesh_tools
import numpy as np
import yaml

ACROSS, ALONG, UP = 0, 1, mesh_tools.UP  # the belt runs along y; x goes across it
FOLDER = os.path.dirname(os.path.abspath(__file__))


def belt_height(triangles, step):
    """Return the height of the belt: the level with the largest upward horizontal area."""
    normals = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
    areas = 0.5 * np.linalg.norm(normals, axis=1)
    flat = (areas > 0.0) & (normals[:, UP] > 0.0) & (np.ptp(triangles[:, :, UP], axis=1) == 0.0)
    levels = np.round(triangles[flat][:, 0, UP] / step) * step
    values, index = np.unique(levels, return_inverse=True)
    return float(values[np.argmax(np.bincount(index, weights=areas[flat]))])


def collision_pieces(triangles, belt, config):
    """Return the convex pieces: the body up to the belt, and one rail on each side of it."""
    points = np.unique(triangles.reshape(-1, 3), axis=0)
    above = points[:, UP] > belt + float(config['rail_above_belt'])
    middle = 0.5 * (points[:, ACROSS].min() + points[:, ACROSS].max())
    pieces = [mesh_tools.convex_hull(points[~above])]
    for side in (points[:, ACROSS] < middle, points[:, ACROSS] >= middle):
        rail = points[above & side]
        if not len(rail):
            raise ValueError('[Fail-Fast] The conveyor has no rail on one of its sides')
        footing = rail.copy()
        footing[:, UP] = belt  # the rail stands on the belt: no gap is left below it
        pieces.append(mesh_tools.convex_hull(np.vstack([rail, footing])))
    return pieces


def top_difference(original, pieces, belt, config):
    """
    Return the largest difference between the top of the pieces and that of the original.

    Where a cut of the original is empty (beside its legs), the pieces only count when they rise
    above the belt.
    """
    check = config['check']
    low, high = original.reshape(-1, 3).min(axis=0), original.reshape(-1, 3).max(axis=0)
    samples = np.arange(low[ACROSS], high[ACROSS], float(check['step']))
    reach = int(round(float(check['across_tolerance']) / float(check['step'])))
    largest = 0.0
    for share in check['cuts']:
        at = low[ALONG] + float(share) * (high[ALONG] - low[ALONG])
        wanted = mesh_tools.top_profile(original, ALONG, at, ACROSS, samples)
        tops = [mesh_tools.top_profile(piece, ALONG, at, ACROSS, samples) for piece in pieces]
        made = np.fmax.reduce(tops)
        covered = ~np.isnan(wanted)
        if np.isnan(made[covered]).any():
            raise ValueError('[Fail-Fast] The collision pieces leave part of the conveyor '
                             'uncovered')
        made = np.where(np.isnan(made), belt, made)
        wanted = np.where(covered, wanted, np.minimum(made, belt))
        shifts = range(-reach, reach + 1)
        nearest = np.min([np.abs(made - np.roll(wanted, shift)) for shift in shifts], axis=0)
        largest = max(largest, float(nearest[reach:len(samples) - reach].max()))
    return largest


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--out', required=True, help='folder the four meshes are written to')
    arguments = parser.parse_args()
    with open(os.path.join(FOLDER, 'conveyor_meshes.yaml'), encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
    source = os.path.join(FOLDER, config['original'])
    original = mesh_tools.read_triangles(source)
    belt = belt_height(original, float(config['level_step']))
    top = float(original[:, :, UP].max())

    visual = config['visual']
    whole = mesh_tools.as_polydata(original, float(config['merge_tolerance']))
    light = mesh_tools.simplified(whole, int(visual['triangles']))
    apart = mesh_tools.deviation(whole, light)
    if apart > float(visual['max_deviation']):
        raise ValueError(f'[Fail-Fast] The simplified conveyor is {apart:.3f} mm from the '
                         f"original: more than visual.max_deviation ({visual['max_deviation']})")

    collision = config['collision']
    pieces = collision_pieces(original, belt, collision)
    difference = top_difference(original, pieces, belt, collision)
    if difference > float(collision['max_deviation']):
        raise ValueError(f'[Fail-Fast] The top of the collision pieces is {difference:.3f} mm '
                         f'from that of the original: more than collision.max_deviation '
                         f"({collision['max_deviation']})")

    os.makedirs(arguments.out, exist_ok=True)
    names = [visual['file'], collision['body_file'], *collision['rail_files']]
    for name, triangles in zip(names, [mesh_tools.as_triangles(light), *pieces]):
        mesh_tools.write_triangles(os.path.join(arguments.out, name), triangles)
    made = os.path.join(arguments.out, names[0])
    megabytes = [os.path.getsize(path) / 1e6 for path in (source, made)]
    print(f'belt at {belt:.1f} mm, rails up to {top:.1f} mm above the floor')
    print(f"| `{config['original']}` | visual | {len(original):,} | {light.GetNumberOfPolys():,} "
          f'| {apart:.3f} | {megabytes[0]:.1f} MB | {megabytes[1]:.2f} MB |')
    for name, piece in zip(names[1:], pieces):
        print(f'| `{name}` | collision, convex | from the original | {len(piece):,} | '
              f'{difference:.3f} (top of the three pieces) | | '
              f'{os.path.getsize(os.path.join(arguments.out, name)) / 1e3:.1f} kB |')


if __name__ == '__main__':
    main()

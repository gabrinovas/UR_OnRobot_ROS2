# Copyright 2026 Tony Le, Gabriel Novas
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Binary STL meshes as arrays: reading, writing, simplifying, convex hulls and their top."""

import numpy as np
from scipy.spatial import ConvexHull
import vtk
from vtk.util.numpy_support import vtk_to_numpy

HEADER_BYTES = 80
RECORD = np.dtype([('normal', '<f4', 3), ('corners', '<f4', (3, 3)), ('attribute', '<u2')])
UP = 2  # the meshes have z up


def read_triangles(path):
    """Return the corners (N, 3, 3) of the triangles of a binary STL, in the units of the file."""
    with open(path, 'rb') as stream:
        stream.seek(HEADER_BYTES)
        count = int(np.frombuffer(stream.read(4), dtype='<u4')[0])
        records = np.frombuffer(stream.read(), dtype=RECORD)
    if len(records) != count:
        raise ValueError(f'[Fail-Fast] {path} is not a binary STL: {len(records)} triangles '
                         f'read, {count} declared')
    return records['corners'].astype(float)


def write_triangles(path, triangles):
    """Write triangles (N, 3, 3) as a binary STL, with the normal of each one."""
    normals = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
    lengths = np.linalg.norm(normals, axis=1, keepdims=True)
    records = np.zeros(len(triangles), dtype=RECORD)
    records['normal'] = normals / np.where(lengths > 0.0, lengths, 1.0)
    records['corners'] = triangles
    with open(path, 'wb') as stream:
        stream.write(b'\0' * HEADER_BYTES)
        stream.write(np.array([len(triangles)], dtype='<u4').tobytes())
        stream.write(records.tobytes())


def convex_hull(points):
    """Return the triangles (N, 3, 3) of the convex hull of some points, facing outwards."""
    hull = ConvexHull(points)
    triangles = points[hull.simplices]
    normals = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
    inwards = np.einsum('ij,ij->i', normals, hull.equations[:, :3]) < 0.0
    triangles[inwards] = triangles[inwards][:, ::-1]
    return triangles


def as_polydata(triangles, merge_tolerance):
    """Return triangles as VTK polydata, with the vertices closer than the tolerance merged."""
    points = vtk.vtkPoints()
    cells = vtk.vtkCellArray()
    for index, corner in enumerate(triangles.reshape(-1, 3)):
        points.InsertNextPoint(*corner)
        if index % 3 == 2:
            cells.InsertNextCell(3, (index - 2, index - 1, index))
    polydata = vtk.vtkPolyData()
    polydata.SetPoints(points)
    polydata.SetPolys(cells)
    clean = vtk.vtkCleanPolyData()
    clean.SetInputData(polydata)
    clean.ToleranceIsAbsoluteOn()
    clean.SetAbsoluteTolerance(merge_tolerance)
    clean.Update()
    return clean.GetOutput()


def as_triangles(polydata):
    """Return the corners (N, 3, 3) of the triangles of VTK polydata."""
    points = vtk_to_numpy(polydata.GetPoints().GetData()).astype(float)
    indices = vtk_to_numpy(polydata.GetPolys().GetConnectivityArray()).reshape(-1, 3)
    return points[indices]


def simplified(polydata, triangles_wanted):
    """Return the polydata simplified to about that many triangles (quadric decimation)."""
    decimate = vtk.vtkQuadricDecimation()
    decimate.SetInputData(polydata)
    decimate.SetTargetReduction(1.0 - triangles_wanted / polydata.GetNumberOfPolys())
    decimate.Update()
    return decimate.GetOutput()


def deviation(first, second):
    """Return the largest distance from a vertex of one surface to the other surface."""
    largest = 0.0
    for origin, target in ((first, second), (second, first)):
        distance = vtk.vtkDistancePolyDataFilter()
        distance.SetInputData(0, origin)
        distance.SetInputData(1, target)
        distance.SignedDistanceOff()
        distance.ComputeSecondDistanceOff()
        distance.Update()
        values = vtk_to_numpy(distance.GetOutput().GetPointData().GetArray('Distance'))
        largest = max(largest, float(values.max()))
    return largest


def top_profile(triangles, along, at, across, samples):
    """
    Return the highest point of a mesh over each sample of one axis, on one cutting plane.

    The plane is where the axis `along` is `at`; samples the mesh does not cover come back as
    nan.
    """
    low, high = triangles[:, :, along].min(axis=1), triangles[:, :, along].max(axis=1)
    heights = np.full(len(samples), np.nan)
    for triangle in triangles[(low <= at) & (high >= at) & (high > low)]:
        crossings = []
        for first, second in ((0, 1), (1, 2), (2, 0)):
            a, b = triangle[first], triangle[second]
            if (a[along] - at) * (b[along] - at) <= 0.0 and a[along] != b[along]:
                crossings.append(a + (at - a[along]) / (b[along] - a[along]) * (b - a))
        if len(crossings) < 2:
            continue
        start, end = sorted(crossings[:2], key=lambda point: point[across])
        inside = (samples >= start[across]) & (samples <= end[across])
        if not inside.any():
            continue
        span = end[across] - start[across]
        level = np.full(int(inside.sum()), max(start[UP], end[UP])) if span <= 0.0 else \
            start[UP] + (samples[inside] - start[across]) / span * (end[UP] - start[UP])
        heights[inside] = np.fmax(heights[inside], level)
    return heights

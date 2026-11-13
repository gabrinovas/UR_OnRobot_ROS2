# Original meshes

The meshes of `meshes/` listed below were simplified on 2026-11-13 so that RViz, robot_state_publisher and MoveIt
load them quickly. This folder keeps the originals, with the same relative paths, and is not installed.
To restore one, copy it back over the file of the same name in `meshes/`.

How they were simplified: VTK 9.1 quadric decimation (vtkQuadricDecimation) after merging duplicate vertices.
The deviation is the largest distance between the original and the simplified surface, measured from every
vertex of each one to the other (the meshes are in millimetres). Limits used: 0.5 mm for visual meshes and
2 mm for collision meshes.

| Mesh | Use | Triangles before | Triangles now | Deviation (mm) | Size before | Size now |
|---|---|---|---|---|---|---|
| `environment/conveyor.stl` | visual | 1,002,854 | 39,998 | 0.357 | 47.8 MB | 1.91 MB |

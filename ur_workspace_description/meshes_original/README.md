# Original meshes

This folder keeps the meshes as they were measured or drawn, and the program that makes from them the meshes of
`meshes/`, which are light enough for RViz, robot_state_publisher and MoveIt to load quickly. It is not installed.

## The conveyor

`environment/conveyor.stl` is the conveyor of the cells `left` and `right` as measured in the lab, in millimetres
and with its origin at a corner on the floor: the belt is 772 mm above the floor and the two rails reach 810 mm.
With the tables 750 mm tall, that is 22 mm and 60 mm above the robot base.

    python3 meshes_original/conveyor_meshes.py --out meshes/environment

makes the four meshes the description of the cell loads, with the values of `conveyor_meshes.yaml`:

- **`conveyor.stl`, to look at.** The original after merging duplicate vertices and a quadric decimation
  (vtkQuadricDecimation, VTK 9.1).
- **`conveyor_body.stl`, `conveyor_rail_1.stl` and `conveyor_rail_2.stl`, to collide with.** Three convex pieces:
  the conveyor up to its belt, and each rail standing on the belt. They are three because a collision checker may
  fill a mesh up to its convex hull (MuJoCo always does), and one piece would close the gap between the rails,
  where the gripper works.

The program checks its own result, and writes nothing when a check fails:

| Check | Limit |
|---|---|
| Largest distance between the original surface and the simplified one, from every vertex of each to the other | 0.5 mm |
| Largest difference between the top of the three pieces and the top of the original, on five cuts across the conveyor | 2 mm |

It needs numpy, scipy, yaml and vtk. The MuJoCo simulation of the same cells loads copies of these four files, so
both see the same conveyor: after making them again, copy them there too.

| Mesh | Use | Triangles before | Triangles now | Deviation (mm) | Size before | Size now |
|---|---|---|---|---|---|---|
| `conveyor.stl` | visual | 1,002,862 | 40,000 | 0.358 | 50.1 MB | 2.00 MB |
| `conveyor_body.stl` | collision, convex | from the original | 652 | 1.297 (top of the three pieces) | | 32.7 kB |
| `conveyor_rail_1.stl` | collision, convex | from the original | 19,768 | 1.297 (top of the three pieces) | | 988.5 kB |
| `conveyor_rail_2.stl` | collision, convex | from the original | 13,098 | 1.297 (top of the three pieces) | | 655.0 kB |

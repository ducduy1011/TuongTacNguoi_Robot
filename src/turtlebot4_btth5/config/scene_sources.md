# Eight 3-D objects for the maze (4 people, 4 chairs)

Downloaded by `tools/prepare_scene.py`, Fuel revision 3:

- **Standing person**: OpenRobotics / Marina Kollmitz, made with MakeHuman.
  https://fuel.gazebosim.org/1.0/OpenRobotics/models/Standing%20person
  License: CC0 1.0, https://creativecommons.org/publicdomain/zero/1.0/
- **Chair**: OpenRobotics / Cole Biesemeyer.
  https://fuel.gazebosim.org/1.0/OpenRobotics/models/Chair
  License: CC BY 4.0, https://creativecommons.org/licenses/by/4.0/

Local changes: set models static and resolve mesh URIs to local absolute files.
Meshes/textures are otherwise unchanged. Downloads live in `.models/`, outside Git.

Default placements in Gazebo `maze` (the TurtleBot 4 maze world loaded by our launch):

| Model | world x | world y | yaw | Approximate map x/y |
|---|---:|---:|---:|---|
| person | -6.0 | -0.5 | -2.15 | -4.0, -0.5 |
| chair | -6.3 | -6.6 | -0.82 | -4.3, -6.6 |
| person_2 | -8.5 | 2.0 | 0.0 | -6.5, 2.0 |
| chair_2 | -7.5 | 3.5 | -1.57 | -5.5, 3.5 |
| person_3 | -2.0 | -8.8 | 1.57 | 0.0, -8.8 |
| chair_3 | -0.2 | -8.8 | 1.57 | 1.8, -8.8 |
| person_4 | 6.0 | -7.3 | 3.14 | 8.0, -7.3 |
| chair_4 | 8.5 | -6.6 | 3.14 | 10.5, -6.6 |

The earlier northern route was retired after a real trial: Goal 1 succeeded,
then AMCL uncertainty increased and Goal 2 aborted with no collision-free local
trajectory. The old raster does not show the northern perimeter consistently.
The previous central/southern goals had about 0.95 m clearance from static
occupied/unknown cells. Offline paths exist after 0.65 m static inflation and
0.90 m exclusion around the eight object centres. These offline checks do not
certify an actual Gazebo navigation run.

The unchanged placements and previous central/southern goals were also tested in a real
Gazebo session on 2026-10-19: the one-command demo completed all three goals in
order, and camera inference detected both person and chair. See the README for
the observed results and limitations; record a fresh video for submission.

On 2026-10-21 the student supplied three different Publish Point goals, listed
in `goals.yaml` and the README. Nav2 computed paths for all three legs without
moving the robot. The new route uses narrower passages; the previous successful
navigation recordings do not validate navigation through these new goals.

The map/world conversion is approximate for the existing week-04 map and default
spawn, not a general TF calibration. Confirm placement and camera visibility in
Gazebo/RViz. These are real obstacles with collisions and LiDAR returns; the
unchanged old map will not contain them. Nav2 must avoid them using live costmaps.
Do not put objects on goals or block narrow corridors. Detection quality on
synthetic meshes must be checked in the actual camera feed; models are not
guaranteed to be recognized in every view. A fire extinguisher is not among
YOLOv8n's default COCO classes; do not mislabel a generic box as an extinguisher.

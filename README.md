# SyncAI-Robot-Landmark-Detection

`syncai_landmark_detection`, a ROS 2 Humble package (C++17, `rclcpp`): where a known landmark is
right now, as seen by the robot's own sensors — for the last metre of a charge, where a dock pose
from the map is not precise enough.

`landmark_detector` serves `syncai_common/srv/DetectLandmark` on `<robot_id>/detect_landmark`. The
detector **only detects**: it returns the landmark's pose and how sure the sighting is, never
commands motion and never decides whether a sighting is good enough. The charge tree in
SyncAI-Robot-Workspace polls it and hands the target to the controller server's `DockToPose`.

Detection methods are pluginlib plugins, picked per landmark by the node's landmark registry; ArUco
is the first, ported from SyncAI-Robot-Charger's `detector_node`.

Status: repository set up; no code yet.

## Build and test

In the Dev Container (VS Code *Reopen in Container*, or `devcontainer up --workspace-folder .`),
which vcs-imports `dependencies.repos` (`syncai_common`) next to the package and builds once:

```bash
docker exec syncai-robot-landmark-detection bash -lc 'cd /workspace && colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON'
docker exec syncai-robot-landmark-detection bash -lc 'cd /workspace && colcon test --packages-select syncai_landmark_detection && colcon test-result --test-result-base build/syncai_landmark_detection --verbose'
```

On the robot the package is vcs-imported into SyncAI-Robot-Workspace as
`src/syncai_landmark_detection` and built with the rest of the workspace.

## Licence

MIT, see `LICENSE`.

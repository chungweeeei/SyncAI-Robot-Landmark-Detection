# CLAUDE.md

The ROS 2 package `syncai_landmark_detection` (C++17, `rclcpp`): landmark detection for the last
metre of a charge. One node, `landmark_detector`, serving `syncai_common/srv/DetectLandmark`.
Set up on 2026-11-09; so far the plugin interface only. The server (registry, service, standby,
transform) and the ArUco plugin come next.

**The detector only detects.** It returns where a landmark is (6-DOF, in `target_frame`, transformed
at the image stamp) and facts about the sighting (`age`, `consecutive`, `ambiguous`,
`reprojection_error_px`). It never commands motion, never judges a sighting beyond `max_age`, and
returns the landmark, not the dock: the tag -> dock offset and every threshold are the caller's
(SyncAI-Robot-Workspace's charge tree, which polls this service and preempts `DockToPose`).
`srv/DetectLandmark.srv` in SyncAI-Robot-Interface is the design record; read it before changing
behaviour.

**Detection methods are pluginlib plugins**, one per method (ArUco first, ported from
SyncAI-Robot-Charger's `detector_node`: `MarkerPose`, `BranchTracker`). The caller names a landmark,
never a method: the node's landmark registry (its params file) maps each `landmark_id` to a plugin
and the landmark's own parameters (ArUco: dictionary, marker id, printed size — the size is per
landmark).

**Plugins are made the way SyncAI-Robot-Workspace's planner and controller plugins are**
(`syncai_nav_core`): the abstract class `syncai_landmark_detection::LandmarkDetector`
(`include/syncai_landmark_detection/landmark_detector.hpp`, header-only and exported), loaded with
`pluginlib::ClassLoader` from `detector_plugins` + `<id>.plugin`, `initialize(node, name, tf)` on a
plain `rclcpp::Node`, parameters under `name.` with `declare_parameter_if_not_declared`, a
`<name>_plugin.xml` exported with `pluginlib_export_plugin_description_file`, and
`PLUGINLIB_EXPORT_CLASS` in the plugin's `.cpp`. Two additions, each documented in the header:
`addLandmark()` (one instance serves several landmarks) and `activate()` / `deactivate()` (a
detector reads its sensor only while polled; not lifecycle). Split of work: a plugin owns its sensor
subscriptions and keeps the newest sighting per landmark in the sensor frame; the server answers
DetectLandmark, judges `max_age` and does the transform. `node_utils.hpp` copies `syncai_util`'s two
parameter helpers rather than depend on that workspace package.

**The camera has one opener** on the robot, `syncai_camera_feeder`. This package must never open the
V4L2 device; it reads what the feeder publishes.

**The external interface is a cross-repo contract**: the package name, the executable and node name,
the `detect_landmark` service (typed by `syncai_common`) and the parameters are used by
SyncAI-Robot-Workspace's session specs and BT nodes. Do not rename any of them casually.

**The repo root is the package itself** (`package.xml` and `CMakeLists.txt` at the root), meant to be
vcs-imported into the Workspace as `src/syncai_landmark_detection`. `.devcontainer/` and
`dependencies.repos` are only for developing this repo on its own.

## Build and test

Everything runs in the Dev Container `syncai-robot-landmark-detection` (VS Code Reopen in Container,
or `devcontainer up --workspace-folder .`). `/workspace` is the named volume
`syncai-robot-landmark-detection_workspace`, this repo is mounted at
`/workspace/src/syncai_landmark_detection`, and `postCreateCommand` vcs-imports `dependencies.repos`
(`syncai_common`) next to it. The host has no ROS or OpenCV contrib: nothing compiles there.

```bash
docker exec syncai-robot-landmark-detection bash -lc 'cd /workspace && colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON'
docker exec syncai-robot-landmark-detection bash -lc 'cd /workspace && colcon test --packages-select syncai_landmark_detection && colcon test-result --test-result-base build/syncai_landmark_detection --verbose'
```

- If `syncai_common` is missing (a fresh volume):
  `cd /workspace && vcs import --skip-existing < src/syncai_landmark_detection/dependencies.repos`.
- CI (`.github/workflows/ci.yml`) runs a clang-format 14 check on an ubuntu-22.04 runner and
  `.github/scripts/ci.sh` (vcs import, colcon build, colcon test with ament_lint_auto) inside the
  image built from `.devcontainer/Dockerfile`. Keep the script and this list in step.

## Dependencies

- OpenCV from apt: `libopencv-dev` + `libopencv-contrib-dev` (cv::aruco). Ubuntu 22.04's 4.5.4 has
  the pre-4.7 aruco API (`cv::aruco::detectMarkers`, not `cv::aruco::ArucoDetector`).
- SyncAI-Robot-Workspace's image must carry the same packages: keep its `Dockerfile` in step when a
  dependency is added here.
- `syncai_common` is a colcon package from `dependencies.repos`; keep the pin in step with the
  Workspace's `dependencies.repos`.

## Real-robot safety

The dev container uses the host network and `ROS_DOMAIN_ID=2`. The detector commands nothing, but a
test against a bag or a camera should still `export ROS_DOMAIN_ID=77` so nothing reaches a real
robot's graph.

## Conventions

- Everything is written in English (code, comments, logs, docs). Log messages carry a `[Module]`
  prefix (e.g. `[LandmarkDetector]`, `[ArucoDetector]`).
- Comments explain **why**, and record the measurement or the rejected alternative behind a value,
  as in SyncAI-Robot-Workspace.
- Formatting follows `.clang-format` (ROS 2 / Google based, copied from SyncAI-Robot-Workspace).
- Commits use Conventional Commits (`.github/prompt/copilot-commit-message-instructions.md`), in
  English.

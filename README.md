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

Plugins follow SyncAI-Robot-Workspace's planner / controller plugins: an abstract
`syncai_landmark_detection::LandmarkDetector` (`include/syncai_landmark_detection/landmark_detector.hpp`)
loaded with pluginlib from `detector_plugins` / `<id>.plugin`. A plugin owns its sensor and keeps the
newest sighting per landmark; the server answers the service and transforms.

## Running it

```bash
ros2 launch syncai_landmark_detection landmark_detector.launch.py
ros2 service call /<robot_id>/detect_landmark syncai_common/srv/DetectLandmark "{landmark_id: charger_a}"
```

The first call wakes the detector and answers `SENSOR_STARTING` (3); ask again. A detector nobody
calls for `idle_timeout` (5 s) stops reading its sensor. The launch file takes `robot_id` from
`config/system.ini` as the namespace, so the plugin reads `syncai_camera_feeder`'s
`<robot_id>/image_raw/compressed` and `<robot_id>/camera_info`. `params/landmark_detector_params.yaml` holds the plugins and the landmark registry
(`landmarks`, one entry per print, each naming its `detector`).

What the server answers: `UNKNOWN_LANDMARK` (not in the registry); `SENSOR_STARTING` (just woken,
or no frame / `camera_info` yet); `SENSOR_UNAVAILABLE` (no frame for `frame_timeout`, no
`camera_info` — the feeder publishes none without its calibration file — a calibration for another
resolution, an undecodable frame); `NOT_DETECTED` (no sighting newer than `max_age`);
`TRANSFORM_FAILED`; otherwise the landmark's pose, stamped with the image's capture time. With
`target_frame` empty it is in the camera's own frame, untransformed
(`<robot_id>/camera_optical_frame`: z ahead, x right, y down), which needs no TF and works in every
mode; any other frame is transformed at that stamp (`<robot_id>/base_link`: relative to the robot;
`<robot_id>/odom`: fixed, AUTO only).

## The ArUco plugin

`syncai_landmark_detection/ArucoDetector`. Subscribes, only while active, to
`image_raw/compressed` (SensorData QoS, decoded straight to grey) and `camera_info` (reliable +
transient local: the feeder publishes it once, latched, so it is cached and never paired with an
image by stamp). Every frame: the markers searched around where they were last seen (the whole
frame when that misses), the outline picked, its corners undistorted with `camera_info`
(`equidistant` as fisheye, else `plumb_bob` / `rational_polynomial`), both PnP solutions, the
choice, the gate.

The detection logic is SyncAI-Robot-Charger's `detector_node`, with its measured values
(`plugins/aruco_detector/*` and the params file have the why of each): sub-pixel corners in a
2 px window, `minMarkerDistanceRate` 0.01, the duplicate / white-line outline rule, the geometry
check (|Rz| <= 60 deg, tilt <= 45 deg), the decisive image (3x + 0.3 px), the LM-refined pose
below 0.6 m (out above 0.65 m) with two meeting refinements counted as decisive, the previous
normal as the hint, the gate against mirror jumps (0.302 m / 20 deg, 5 in a row to relock, reset
after 0.5 s). Its geometry and choice are ROS-free (`aruco_core`), covered by
`test/test_marker_geometry.cpp` and `test/test_solution_selector.cpp`.

Not ported, on purpose:
- **The motion-based choice** (the charger's `BranchTracker`, gyro + Point-LIO). Two solutions the
  image cannot tell apart are reported `ambiguous: true`, picked by the previous normal; the caller
  decides. Expect it at ~1 m, rarely below 0.6 m. It can come back later from TF, not new
  subscriptions.
- **The smoothing.** Only the gate stays: smoothing in the camera frame lags and bends while the
  robot moves; it belongs to whoever holds the pose in a fixed frame.
- **The stream handling**: GStreamer, NV12, the detect-rate throttle, filling a missed frame from
  the ones before, latency statistics, debug images, `pose_raw`, the field-of-view pinhole and the
  calibration scaling (`camera_info` replaces them), `marker_mirrored`, switching `marker_id` live
  (a second print is a second registry entry).

Output differs from the charger's: the marker's 6-DOF pose in the camera frame at the capture stamp,
transformed by the server, where the charger published the camera's (X, Y, Rz) in the marker frame,
smoothed, stamped on arrival.

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

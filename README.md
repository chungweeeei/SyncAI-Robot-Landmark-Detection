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
(`landmarks`, one entry per print, each naming its `detector`). Two entries with the same ArUco
dictionary and `marker_id` stop the node at startup, whatever their `marker_inverted`: an inverted
search decodes normal prints too, so one print would answer for both.

What the server answers: `UNKNOWN_LANDMARK` (not in the registry); `SENSOR_STARTING` (just woken,
or no frame / `camera_info` yet); `SENSOR_UNAVAILABLE` (no frame for `frame_timeout`, no
`camera_info` — the feeder publishes none without its calibration file — a calibration for another
resolution, an undecodable frame, a frame OpenCV threw on; cleared by the next good frame);
`NOT_DETECTED` (no sighting newer than `max_age`, or one in an image stamped more than `max_age`
ahead of the node's clock — the two clocks disagree; less than that ahead answers `age` 0);
`TRANSFORM_FAILED`; otherwise the landmark's pose, stamped with the image's capture time. A
`NOT_DETECTED` says which of the plugin's reasons the newest frame had (`marker id 42 not found in
the frame (0 other marker(s) of its dictionary were); exposure searching at 4200 us`, `... found,
its pose dropped by the gate against mirror jumps; exposure locked at 1700 us`), and every answer
that is not the landmark, `SENSOR_STARTING` aside, is also a warning in the node's own log, at most
one per 5 s. With
`target_frame` empty it is in the camera's own frame, untransformed
(`<robot_id>/camera_optical_frame`: z ahead, x right, y down), which needs no TF and works in every
mode; any other frame is transformed at that stamp (`<robot_id>/base_link`: relative to the robot;
`<robot_id>/odom`: fixed, AUTO only).

## The ArUco plugin

`syncai_landmark_detection/ArucoDetector`. Subscribes, only while active, to
`image_raw/compressed` (SensorData QoS at depth 1, so a frame slower than the 30 Hz period drops
the ones behind it rather than queueing them; decoded straight to grey) and `camera_info` (reliable +
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
accepted normal as the hint, the gate against mirror jumps (0.302 m / 20 deg, 5 in a row to relock,
reset after 0.5 s). Two changes of ours: the refinement is not tried while the IPPE pose is past
0.70 m (it would be thrown away), and a pose the gate drops never becomes the hint (it did, and
re-locked the gate onto the mirror). Its geometry and choice are ROS-free (`aruco_core`), covered by
`test/test_marker_geometry.cpp` and `test/test_solution_selector.cpp`.

### The camera's exposure

While it is awake the plugin sets the camera's exposure (`ArUco.exposure.*`). The camera's auto
exposure meters the whole frame and goes no shorter than 8333 µs; the dock's marker is 0.4 % of the
frame, so where it is brighter than the room it burns out. On robot01 (2026-11-11, in front of the
dock, auto exposure at 23 400 µs) 51 % of the marker's pixels were at 250 or above, its black cells
grey at ~150 and its black border gone, and nothing was detected. The detector is who knows where
the marker is, so it decides; **`syncai_camera_feeder` still does every write** (its
`exposure_override_us` parameter, microseconds, 0 = its own configured exposure), so the camera
keeps one opener and one writer.

```
activate()  -> exposure_override_us = 8333                        SEARCHING
  no marker for miss_frames  -> next of 8333, 4166, 2083, 1000, 16600 (and round again)
  marker, region burnt out   -> exposure x 0.5
  marker, p95 outside band   -> exposure x target_p95 / p95   (at most 2x either way)
  marker, inside the band    ->                                    LOCKED: hold
      burnt out or dark past the wider unlock margins, 3 frames in a row -> adjust, SEARCHING
      no marker for lost_frames -> the ladder, from the rung nearest the held exposure
deactivate() -> exposure_override_us = 0
```

- **It holds the exposure once the marker reads well**, as the charger held one for a whole
  alignment: a change of exposure moves where the corners are found, and the caller is steering on
  that pose. The band it locks in (p95 160–220, ≤ 2 % saturated) is narrower than the margins it
  leaves on (p95 < 100, > 10 % saturated), so the exposure stays still while the robot walks up to
  the marker.
- **The region metered** is the marker's quad grown by 1.25 (`camera_exposure/region_brightness.*`),
  to take in the print's white rim, which burns out first and takes the border with it. Its 95th
  percentile is the white cells' level whatever share of it is black.
- **Frames just after a change decide nothing** (`settle_frames`): they were exposed before it.
  They are still searched, and a sighting in one is still answered.
- **A failed write never fails a detection.** No feeder at wake-up (a bag replay, a Dev Container),
  or no answer within `set_timeout`: one warning, and the exposure is left alone until the next
  wake-up. A refused value: a warning, the next step is still tried. A failed hand-back at standby
  is an error that names the exposure the camera is left at and the command that undoes it.
- **Every other viewer of the camera gets the detector's picture** while it is awake (the console,
  a bag recording): darker, and changing by the rung every 0.27 s while no marker is in view.
- **Limits.** Nothing gives the camera back if this node dies while awake (its launch does not
  respawn it): the feeder keeps the last exposure until it restarts or someone sets
  `exposure_override_us` to 0. A marker that does not read at the ladder's shortest rung is never
  metered, so nothing takes the exposure below that rung. Another writer of the camera's controls
  (SyncAI-Robot-Charger's `camera_control.py`) is not seen: stop it. And **no value here is
  measured on the camera yet** — how many frames a change takes to show, the marker's brightness
  against the exposure, the frame rate over minutes of manual exposure (the charger saw it step
  down to 30 Hz through `vizion-ctl`, cause unknown).

The decision is ROS-free (`camera_exposure_core`: `ExposurePolicy`, `measureRegion`), covered by
`test/test_exposure_policy.cpp` (the ladder, convergence on a simulated sub-linear camera from a dim
marker to a sunlit one, the lock, the limits) and `test/test_region_brightness.cpp`. Its parameters
reach it through `declareExposurePolicyParameters`, which `test/test_exposure_parameters.cpp` runs
on a plain node: a ladder read through a range-for over the temporary parameter's array started
every search at 16600 µs, or stopped the node starting when a rung came out negative, until
2026-11-11, so run that test under AddressSanitizer when this path changes.
`FeederExposureClient` is the one piece that talks to the feeder. `exposure.enabled: false` leaves
the camera to the feeder.

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

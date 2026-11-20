# Saved robot and camera calibration

These files are committed to Git. A reboot or application restart does **not** require repeating
calibration. The laptop's `config.yaml` selects the active lens/floor files; wheel services use their
robot-specific configuration. Pending head measurements are retained here but are not enabled as a
completed navigation calibration.

| File | Saved information / status |
| --- | --- |
| [camera_intrinsics.yaml](camera_intrinsics.yaml) | Lens matrix and distortion at 4608 × 2592; 12 views, 0.874 px RMS. |
| [floor_pose_36cm.yaml](floor_pose_36cm.yaml) | Current floor transform. Owner measured 39.5 cm lens height and ~103 cm floor-tag spacing; camera fit 37.9 cm and 103.6 cm. Filename is historical. |
| [head_neutral_pending.json](head_neutral_pending.json) | Neutral capture and sourced physical measurements; **incomplete for navigation**. |
| [../robot/config/robot1_three_wheels.json](../robot/config/robot1_three_wheels.json) | Robot 1 wheel mapping/signs, M1 disabled, six-count speed cap. |
| [../robot/config/robot2_wheels.json](../robot/config/robot2_wheels.json) | Robot 2 wheel mapping and board settings. |
| [floor_pose.yaml](floor_pose.yaml) | Historical camera pose at about 1 m; not the active floor transform. |

## Head reference measurements, 2026-11-20

Chassis coordinates: origin on the floor below the midpoint of the four wheel centers; +X forward,
+Y left, +Z up. Units below are meters unless stated otherwise.

- Both heads were placed level and facing their chassis fronts. Top/front tags are not needed for this
  reference capture. Remaining robot black-square markers are **10 cm**; temporary floor markers are 15 cm.
- Rear marker IDs: left/Robot 1 = **2**, right/Robot 2 = **6**.
- Owner supplied **14 cm** rear-tag offset, centered left/right. Recorded for both as **X = -0.14 m,
  Y = 0 m**, interpreting “rear” as behind chassis center. Offset is approximate, not a precision survey.
- The left head is mechanically secured; its servo interface does not respond. Do not assume servo
  feedback or a torque command can verify its fixed position.
- Right neutral servo capture: **pan 391, tilt 770**, torque off at capture. These are reference readings,
  not a command to return there automatically or a verified engaged hold.
- Camera estimates of rear-tag center heights were 0.295 m (left) and 0.294 m (right). These are camera
  estimates and remain separate from owner-measured dimensions. Raw PnP alternatives are retained.

Remaining work: validate marker heights; measure right pan/tilt axis locations and marker mount hierarchy;
establish joint direction/angle scale; verify clock alignment and joint-feedback transport; check that
estimated chassis position/heading remain stable as the right head moves. Driving stays disabled until
its required pose is validated. See [HEAD_TRACKER.md](../docs/HEAD_TRACKER.md).

## When to check calibration again

- After a reboot, check that the selected calibration files loaded, marker IDs are correct, the left head
  remains secured, and live feedback/clock checks pass. Do not automatically overwrite saved measurements.
- Moving the main camera changes the floor transform: redo floor calibration. Lens changes, focus changes,
  resolution changes or cropping may also require lens-calibration validation.
- Moving a tag, reindexing a servo horn, changing a linkage, or shifting a secured head invalidates the
  affected head geometry. Ordinary robot travel or a reboot does not invalidate the fixed dimensions.
- IDs **0 and 1** are currently excluded from robot poses because borrowed copies are on the floor.
  Remove those floor copies before reusing those IDs on robots. Duplicate IDs must not be averaged.

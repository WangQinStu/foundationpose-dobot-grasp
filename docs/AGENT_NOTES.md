# Agent Notes for foundationpose-dobot-grasp

This file is for future Codex/agent runs. Read it before changing this
workspace. The project is a full visual servo-style bottle grasping workspace:
RealSense RGB-D and YOLO choose the bottle, FoundationPose estimates 6D pose,
optional SDFR refines it, then a UDP/ROS2 bridge sends a Dobot TCP target to a
pick/place state machine that controls both arm motion and the gripper.

## Repository Map

- `FoundationPose/`
  - Main vision workspace. The real-time bottle pipeline lives in
    `FoundationPose/yolo_foundationpose/`.
  - Entry point: `FoundationPose/run_realsense_yolo_foundationpose.py`.
  - There is also `FoundationPose/AGENTS.md` with FoundationPose-specific
    development notes.
- `FoundationPose/yolo_foundationpose/`
  - `app.py`: real-time main loop.
  - `config.py`: CLI options for camera, YOLO, FoundationPose, SDFR, ROS pose
    publishing, and Dobot target publishing.
  - `camera.py`: starts RealSense, aligns depth to color, returns BGR/RGB/depth
    and camera matrix `K`.
  - `foundation.py`: loads mesh and builds `FoundationPose`, scorer, refiner,
    CUDA rasterizer, OBB transform, and bbox.
  - `masks.py`: cleans YOLO masks and optionally masks RGB-D input before
    FoundationPose register/track.
  - `target_lock.py`: sticky target selection so multi-bottle scenes do not jump
    targets on YOLO score changes.
  - `pose_refiner.py`: optional SDFR adapter.
  - `dobot_bridge.py`: conda-safe UDP bridge from visual pose to Dobot TCP
    target.
  - `ros_pose_publisher.py`: direct `rclpy` publisher for `ob_in_cam`; useful
    only when Python/ROS ABI matches.
- `FoundationPose/yolo_foundationpose/assets/`
  - Current default YOLO/FoundationPose resources. Default mesh is
    `mesh/bottle_cad2.obj`; default weights are
    `weights/best_1.pt`.
- `SDFR/`
  - Optional pose refinement code. The online integration dynamically loads
    `SDFR/my_tools/inference.py` and calls `refine_pose_with_sdfr(...)`.
- `dobot/`
  - ROS2 workspace for Dobot bringup, MoveIt assets, WHEELTEC step motor
    gripper, and the business logic package `dobot_pick_place`.
- `dobot/src/dobot_pick_place/`
  - `target_pose_gripper.py`: receives target TCP pose, initializes robot if
    needed, sends MovJ/MovL, controls gripper, and optionally transfers bottle
    through intermediate/place/camera poses.
  - `foundationpose_udp_bridge.py`: ROS-side UDP JSON bridge.
  - `current_pose_publisher.py`: converts Dobot `ToolVectorActual` from
    millimeters plus RPY degrees into `/dobot_pick/current_pose` as meters plus
    quaternion.
  - `launch/target_pose_gripper.launch.py`: starts target controller, current
    pose publisher, UDP bridge, and gripper serial node by default.

## Main Data Flow

```text
RealSense D435i color/depth
  -> depth aligned to color and clipped to zmin/zmax
  -> YOLO segmentation
  -> BottleTargetSelector scores candidate bottles
  -> TargetLock keeps the selected bottle stable
  -> mask refinement and optional RGB-D masking
  -> FoundationPose register() or track_one()
  -> optional SDFR refinement of the same ob_in_cam pose
  -> visualization, pose saving, optional ROS pose publishing
  -> DobotTargetBridge converts ob_in_cam to Dobot TCP target
  -> UDP 127.0.0.1:5005
  -> ROS-side foundationpose_udp_bridge
  -> /dobot_pick/target_pose
  -> target_pose_gripper state machine
  -> Dobot MovJ/MovL services and /motor_control gripper commands
```

## Runtime Entrypoints

Vision:

```bash
cd /home/ptcs/wkSpace/foundationpose-dobot-grasp/FoundationPose
python run_realsense_yolo_foundationpose.py
```

Vision with Dobot target publishing:

```bash
python run_realsense_yolo_foundationpose.py \
  --dobot_publish_target \
  --dobot_eye_in_hand 1
```

Dobot bringup, from the ROS2 workspace:

```bash
cd /home/ptcs/wkSpace/foundationpose-dobot-grasp/dobot
source install/setup.bash
ros2 launch cr_robot_ros2 dobot_bringup_ros2.launch.py
```

Dobot pick/place controller and UDP bridge:

```bash
cd /home/ptcs/wkSpace/foundationpose-dobot-grasp/dobot
source install/setup.bash
ros2 launch dobot_pick_place target_pose_gripper.launch.py
```

Useful topic checks:

```bash
ros2 topic echo /dobot_pick/sequence_state --once
ros2 topic echo /dobot_pick/current_pose --once
ros2 topic echo /dobot_pick/target_pose --once
```

## Coordinate and Unit Conventions

- FoundationPose pose is `ob_in_cam`: object frame to camera optical frame,
  represented as a 4x4 matrix in meters.
- FoundationPose/OpenCV matrix convention is used in this project; keep an eye
  on `glcam_in_cvcam = diag(1,-1,-1,1)` in FoundationPose internals.
- The Dobot target topic `/dobot_pick/target_pose` uses ROS `PoseStamped`:
  position in meters and quaternion `xyzw`.
- Dobot driver MovL/MovJ pose services expect position in millimeters and
  orientation as RPY degrees. `target_pose_gripper.py` performs this conversion.
- `current_pose_publisher.py` converts Dobot feedback
  `/dobot_msgs_v4/msg/ToolVectorActual` from millimeters/RPY degrees back to
  meters/quaternion on `/dobot_pick/current_pose`.
- The default `bottle_cad2.obj` height/long axis is object-frame `Y`. Its
  geometric center is about `Y=0.0675 m`, so the default grasp offset is
  `--dobot_grasp_offset_obj 0,0.0675,0`.

## Dobot Target Conversion

`yolo_foundationpose/dobot_bridge.py` receives the visual `ob_in_cam` pose and
publishes a TCP target only when the robot is idle and the pose is stable.

Important behavior:

- It uses UDP JSON rather than direct `rclpy` because ROS2 Jazzy `rclpy` is often
  compiled for system Python 3.12, while FoundationPose commonly runs in a conda
  Python environment.
- Fixed-camera mode needs `--dobot_base_T_cam_file` or `--dobot_base_T_cam`.
- Eye-in-hand mode (`--dobot_eye_in_hand 1`) uses current gripper pose from the
  ROS-side status UDP plus `gripper_T_cam` from
  `--dobot_R_cam2gripper` and `--dobot_t_cam2gripper`.
- The bridge computes `base_T_cam`, then `ob_in_base = base_T_cam @ ob_in_cam`.
- Grasp point:
  - object-frame offset: `dobot_grasp_offset_obj`
  - TCP tool length correction: target TCP position is
    `grasp_xyz - tcp_rotation @ dobot_tcp_to_tip`.
- Orientation modes:
  - `fixed`: use `dobot_grasp_quat_xyzw`.
  - `object`: use full object orientation.
  - `planar`: keep the fixed approach direction and rotate only around the
    approach axis to align the gripper reference axis with the projected bottle
    long axis. This is the default and is usually safer for tabletop bottles.
  - `axis_align`: fully aligns a TCP reference axis to the object long axis.
- It waits for:
  - sequence state in `dobot_idle_states`, default `idle`;
  - `dobot_pose_stable_frames` frames under
    `dobot_pose_max_translation_jitter`;
  - publish cooldown;
  - target distance inside `[dobot_min_target_distance, dobot_max_target_distance]`.

## Dobot Pick/Place State Machine

`target_pose_gripper.py` is the business control node. It subscribes to
`/dobot_pick/target_pose` and publishes `/dobot_pick/sequence_state`.

Typical state flow:

```text
idle -> preparing_pick -> to_pre_grasp -> picking
     -> to_intermediate -> to_place -> to_camera -> idle
```

Important behavior:

- It ignores new targets while `busy=True`.
- It can initialize the robot:
  `RequestControl -> PowerOn -> StopDrag -> ClearError -> EnableRobot ->
  Continue -> SpeedFactor`.
- It inserts a pre-grasp pose above the target, and can use `MovJ` for wrist
  pre-alignment before a short `MovL` descent.
- Dobot driver `MovL.Request.mode` is counterintuitive:
  - `True`: joint mode.
  - `False`: Cartesian pose mode.
  - This project uses `False` because visual targets are TCP poses.
- Gripper is WHEELTEC step motor on `/motor_control`, default serial
  `/dev/ttyACM0`.
- The code uses relative angle mode `mode=2` because the current gripper firmware
  was observed to respond to this mode, while documented absolute mode `mode=4`
  may not work.
- Default launch enables `place_after_grasp=true`, so after closing the gripper
  the arm moves to the default intermediate point, then box/place point, opens,
  and returns to the default camera pose.

## SDFR Integration

SDFR is optional and enabled with `--use_sdfr`.

Online path:

```text
FoundationPose pose
  -> SdfrPoseRefiner
  -> SDFR/my_tools/inference.py:refine_pose_with_sdfr
  -> candidate refined pose
  -> acceptance checks
  -> adopted pose or original FoundationPose pose
```

`SDFR/my_tools/inference.py` currently implements a local SDF + ICP refinement:

- back-project masked depth points with `K`;
- crop around initial pose depth;
- downsample;
- optimize SDF residual with `scipy.optimize.least_squares`;
- optionally polish with Open3D point-to-plane ICP.

`pose_refiner.py` adds safety checks:

- too few masked points -> skip;
- translation jump over `sdfr_accept_max_translation_mm` -> reject;
- rotation jump over `sdfr_accept_max_rotation_deg` -> reject;
- YOLO projected bbox/mask/depth mismatch -> reject when context is provided;
- no enough score improvement -> reject;
- accepted pose updates FoundationPose tracker state with
  `est.set_pose_last_from_full_pose(refined_pose)`.

Known implementation detail to verify before relying on SDFR:

- `SdfrPoseRefiner.refine(...)` has `frame_id=None` and `is_register=False`
  defaults.
- `app.py` currently calls `pose_refiner.refine(color_rgb, depth, ob_mask, pose,
  K, args.mesh_file)` without passing `frame_id`, `is_register`, `best`,
  `to_origin`, or `bbox`.
- Because `--sdfr_register_only` defaults to `1`, this can cause SDFR to report
  `skipped_track_frame` and never actually refine unless that flag is disabled
  or the app call is updated to pass register/track context.
- Because detection context is not passed, the SDFR detection-mismatch rejection
  check is not active in the current call path.

## Vision Pipeline Details

- `BottleTargetSelector` scores YOLO masks by confidence, area, image center,
  border margin, solidity, aspect ratio, depth quality, and depth isolation.
- `TargetLock` keeps the same target by bbox IoU, center distance, and depth.
  Pressing `n` releases the current target; pressing `r` resets both pose and
  target lock.
- `refine_mask_for_pose` filters the mask by valid depth, median-depth band,
  erosion/open/close morphology, and connected-component selection around the
  candidate bbox center.
- `pose_matches_detection` rejects drift by checking projected pose center,
  candidate mask, projected 3D bbox IoU, and depth consistency.
- `track_between_yolo` defaults to off so stale masks do not pull
  `track_one()` back to old image regions.
- Pose saving writes full `ob_in_cam` matrices to
  `debug/realsense_yolo/ob_in_cam/` when `--save_pose` is enabled.

## Common Pitfalls

- Do not use `--ros_publish_pose` for the Dobot closed loop unless the current
  Python environment can import `rclpy`. Use `--dobot_publish_target` and the
  UDP bridge for normal operation.
- Do not use `--dobot_allow_identity_handeye` on the real robot. It is only for
  desk tests.
- If targets are not published, check the Dobot bridge log reason: waiting for
  idle, waiting for current gripper pose, pose jitter too high, cooldown, or
  target distance out of range.
- If the gripper moves the wrong way, change `gripper_close_dir`.
- If the gripper buzzes at limits, reduce close/open angles or set close/open
  durations so a stop command is sent.
- If the robot is slow during long orientation changes, keep the pre-grasp
  MovJ orientation alignment path enabled.
- If SDFR appears enabled but has no effect, inspect the status panel/logs for
  `skipped_track_frame`, missing `refine_pose_with_sdfr`, too few points, or
  rejected jumps.
- Root `README.md` references `docs/PROJECT_STRUCTURE.md` and
  `docs/GITHUB_UPLOAD.md`, but no `docs/` directory was present when this note
  was written.

## Build and Verification Notes

- Python style in this repo is mixed. Existing FoundationPose/yolo integration
  uses compact imports and 2-space indentation; ROS2 Python nodes use standard
  4-space indentation.
- There is no unified test suite. Use the smallest relevant smoke test:
  - vision-only import/compile checks when hardware is unavailable;
  - `ros2 topic echo` for bridge/state topics;
  - real RealSense/Dobot runs for true end-to-end validation.
- Rebuild ROS2 packages after editing `dobot_pick_place`:

```bash
cd /home/ptcs/wkSpace/foundationpose-dobot-grasp/dobot
colcon build --packages-select dobot_pick_place
source install/setup.bash
```

- Do not commit weights, datasets, generated `debug/` output, or build artifacts.
- At the time this note was created, the worktree already had modified files in
  `FoundationPose/yolo_foundationpose/` and `dobot/src/dobot_pick_place/`.
  Treat those as user work unless explicitly told otherwise.

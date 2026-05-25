# FP Robotic Grasping Workspace

This repository is a three-part workspace for RealSense + YOLO + FoundationPose object pose estimation, optional SDFR pose refinement, and Dobot ROS 2 pick/place control.

## Layout

```text
FP/
├── FoundationPose/   # Vision pipeline: RealSense, YOLO segmentation, FoundationPose pose tracking
├── SDFR/             # Optional pose refinement utilities and models
├── dobot/            # ROS 2 workspace for Dobot control and the pick/place bridge
└── docs/             # Project organization and GitHub upload notes
```

The directory names are intentionally kept stable because the current scripts use relative paths between these parts.

## Main Runtime Flow

```text
RealSense RGB-D
  -> YOLO target selection
  -> FoundationPose 6D pose
  -> optional SDFR refinement
  -> UDP target bridge
  -> /dobot_pick/target_pose
  -> Dobot MovL/MovJ + gripper sequence
```

## Typical Startup

Terminal 1, start the Dobot ROS 2 bringup as required by your robot setup.

Terminal 2, start the pick/place controller and UDP bridge:

```bash
cd /home/ptcs/wkSpace/FP/dobot
source install/setup.bash
ros2 launch dobot_pick_place target_pose_gripper.launch.py
```

Terminal 3, run the vision pipeline:

```bash
cd /home/ptcs/wkSpace/FP/FoundationPose
conda activate foundationpose
python run_realsense_yolo_foundationpose.py \
  --dobot_publish_target \
  --dobot_eye_in_hand 1
```

## Useful Checks

```bash
ros2 topic echo /dobot_pick/sequence_state --once
ros2 topic echo /dobot_pick/current_pose --once
ros2 topic echo /dobot_pick/target_pose --once
```

More detailed notes live in:

- [Project Structure](docs/PROJECT_STRUCTURE.md)
- [GitHub Upload Notes](docs/GITHUB_UPLOAD.md)
- [Dobot pick/place README](dobot/src/dobot_pick_place/README.md)
- [Vision bridge README](FoundationPose/yolo_foundationpose/README.md)


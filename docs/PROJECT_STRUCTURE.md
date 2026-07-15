# Project Structure

This workspace keeps only the files needed for the current online bottle grasping flow.

```text
foundationpose-dobot-grasp/
├── FoundationPose/
│   ├── run_realsense_yolo_foundationpose.py
│   ├── yolo_foundationpose/      # RealSense + YOLO + FoundationPose runtime pipeline
│   │   └── assets/
│   │       ├── mesh/bottle_cad2.obj
│   │       └── weights/best_1.pt
│   ├── weights/                  # FoundationPose scorer/refiner checkpoints
│   ├── mycpp/                    # native extension source
│   └── mycpp/build/              # required native extension output
├── SDFR/                         # optional online pose refinement code and model assets
├── dobot/
│   └── src/
│       ├── dobot_pick_place/     # project pick/place bridge and gripper sequence
│       ├── Step_Motor_ROS2/      # WHEELTEC gripper motor bridge
│       └── DOBOT_6Axis_ROS2_V4/  # Dobot driver, messages, bringup
└── docs/
```

Current main commands:

```bash
cd dobot
colcon build
source install/setup.bash
ros2 launch dobot_pick_place target_pose_gripper.launch.py
```

```bash
cd FoundationPose
python run_realsense_yolo_foundationpose.py \
  --dobot_publish_target \
  --dobot_eye_in_hand 1 \
  --dobot_grasp_orientation_mode 3d
```

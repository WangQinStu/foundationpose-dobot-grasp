# Cleanup Notes

Removed local-only or offline-demo files:

- ROS 2 generated outputs: `dobot/build/`, `dobot/install/`, `dobot/log/`
- Python caches: `__pycache__/`
- FoundationPose offline demo datasets: `FoundationPose/demo_data/`
- RealSense bag/demo recordings: `FoundationPose/realsense/data/`
- SDFR rendered evaluation datasets: `SDFR/datasets/render/`, `SDFR/datasets/or/render/`
- SDFR local capture videos and temporary real data
- Duplicate old YOLO and mesh resources not used by the default runtime
- Old `FoundationPose/realsense/` reference mesh/calibration folder
- Upstream offline demo entrypoints: `run_demo.py`, `run_demo1.py`, `run_linemod.py`, `run_ycb_video.py`
- FoundationPose upstream visual assets, Blender drafts, and Docker helper files

Kept because they are required by the current runtime:

- `FoundationPose/yolo_foundationpose/assets/mesh/bottle_cad2.obj`
- `FoundationPose/yolo_foundationpose/assets/weights/best_1.pt`
- `FoundationPose/weights/`
- `FoundationPose/yolo_foundationpose/`
- `dobot/src/dobot_pick_place/`
- `dobot/src/Step_Motor_ROS2/`
- `dobot/src/DOBOT_6Axis_ROS2_V4/dobot_bringup_v4/`
- `dobot/src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/`
- SDFR core code and model files for optional `--use_sdfr`

If generated ROS files are removed, rebuild before running:

```bash
cd /home/ptcs/wkSpace/foundationpose-dobot-grasp/dobot
colcon build
source install/setup.bash
```

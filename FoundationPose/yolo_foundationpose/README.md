# YOLO + FoundationPose 运行结构

这个目录只放 RealSense + YOLO 分割 + FoundationPose 在线位姿估计流程相关代码，避免继续把逻辑堆在单个入口脚本里。

## 文件职责

- `app.py`: 主运行循环，串联相机、YOLO、目标锁定、FoundationPose 和显示窗口。
- `config.py`: 命令行参数和默认路径。
- `camera.py`: RealSense D435i 初始化、对齐彩色图和深度图。
- `compat.py`: 旧版 Ultralytics 与新版 PyTorch 的加载兼容处理。
- `foundation.py`: FoundationPose、scorer、refiner 和物体 mesh 的初始化。
- `geometry.py`: bbox、投影和位姿一致性检查。
- `masks.py`: YOLO mask 的深度过滤、形态学清理，以及给 FoundationPose 的输入裁剪。
- `target_lock.py`: 多目标或目标短暂丢失时的锁定逻辑。
- `visualization.py`: 深度图、分数面板、mask overlay 和位姿坐标轴绘制。

## 入口

仍然使用仓库根目录下的入口：

```bash
python run_realsense_yolo_foundationpose.py
```

常用资源放在：

- `yolo_foundationpose/assets/mesh/`: 当前在线抓取使用的物体 mesh。
- `yolo_foundationpose/assets/weights/`: 当前在线抓取使用的 YOLO 权重。
- `debug/realsense_yolo/`: 运行输出和可选保存的位姿。

## 目标切换相关参数

- `--lock_max_center_ratio`: 旧锁定目标和新 YOLO 候选的最大中心偏移比例，越小越不容易跳到旁边的物体。
- `--lock_switch_after_lost`: 原锁定瓶子连续丢失多少帧后，才允许切换到当前 YOLO 最优目标。
- `--lock_switch_on_miss`: 原锁定瓶子连续丢失达到阈值后，是否切换到当前 YOLO 最优目标，并重置 FoundationPose。
- `--follow_yolo_best`: 是否让当前 YOLO 最优目标优先驱动锁定目标；默认关闭，避免 YOLO 分数跳变导致频繁换瓶子。
- `--prefer_foreground_target`: 多瓶堆叠时是否优先选择离相机最近、最表面的瓶子；默认开启。
- `--foreground_depth_window`: 前景深度窗口，默认 `0.06m`。视觉会先找最浅候选瓶子，只在比它深不超过该窗口的候选里继续按综合分数选择。
- `--foreground_switch_depth`: 已锁定深处瓶子时，如果新候选比当前目标近超过该值，自动切换到前景目标并重置 FoundationPose；默认 `0.06m`。
- `--pose_min_projected_iou`: FoundationPose 3D 框投影和 YOLO 2D 框的最小重叠，用于过滤漂移位姿。
- `--track_between_yolo`: YOLO 间隔帧是否继续调用 FoundationPose track；默认关闭，避免没有新 mask 时把位姿越跟越偏。

## ROS 2 位姿发布

当前主循环里的 `pose` 是 FoundationPose 的 `ob_in_cam`：物体坐标系到相机坐标系的 4x4 变换，单位是米。换句话说，发布出去的 `PoseStamped` 表示“目标物体在相机光学坐标系下的位置和姿态”。机械臂电脑订阅后，通常还需要用手眼标定外参转换到 `robot_base`。

注意：`--ros_publish_pose` 会在 FoundationPose 进程里直接 import `rclpy`。ROS 2 Jazzy 的 `rclpy` 通常是给系统 Python 3.12 编译的，而 FoundationPose conda 环境是 Python 3.10；这种情况下即使 source 了 ROS 环境也会因为 Python ABI 不匹配而失败。实际 Dobot 闭环抓取请用下一节的 UDP bridge，也就是运行视觉端时使用 `--dobot_publish_target`，不要使用 `--ros_publish_pose`。

视觉电脑先 source ROS 2 环境，然后运行：

```bash
source /opt/ros/<distro>/setup.bash
python run_realsense_yolo_foundationpose.py --ros_publish_pose
```

默认发布：

```text
topic: /foundationpose/target_pose
type: geometry_msgs/msg/PoseStamped
frame_id: camera_color_optical_frame
```

如果要改 topic 或 frame：

```bash
python run_realsense_yolo_foundationpose.py \
  --ros_publish_pose \
  --ros_pose_topic /target_pose \
  --ros_pose_frame_id camera_color_optical_frame
```

机械臂电脑在同一个 `ROS_DOMAIN_ID` 和局域网下可以检查：

```bash
ros2 topic echo /foundationpose/target_pose
```

## 直接闭环控制 Dobot

现在可以让视觉主循环直接发布 Dobot 抓取目标：

```text
FoundationPose ob_in_cam
  -> 手眼外参 base_T_cam
  -> Dobot 基座坐标系下的 TCP 目标
  -> /dobot_pick/target_pose
```

Dobot 侧的 `target_pose_gripper` 会发布流程状态：

```text
/dobot_pick/sequence_state
std_msgs/msg/String

idle -> picking -> to_intermediate -> to_place -> to_camera -> idle
```

视觉端默认只在状态为 `idle`、目标位置连续稳定若干帧后发布一次目标。机械臂抓取、放置、回拍照点期间，视觉端会继续显示画面，但不会反复发送新抓取点。

运行前必须准备手眼标定矩阵 `base_T_cam.txt`，内容是 4x4 矩阵，表示相机坐标系到 Dobot 基座坐标系的变换，单位米：

```text
r11 r12 r13 tx
r21 r22 r23 ty
r31 r32 r33 tz
0   0   0   1
```

推荐运行顺序：

```bash
# 终端 1: Dobot bringup
cd /home/ptcs/wkSpace/FP/dobot
source install/setup.bash
ros2 launch cr_robot_ros2 dobot_bringup_ros2.launch.py

# 终端 2: 抓取/放置/回拍照点控制节点
cd /home/ptcs/wkSpace/FP/dobot
source install/setup.bash
ros2 launch dobot_pick_place target_pose_gripper.launch.py

# 终端 3: 视觉闭环
cd /home/ptcs/wkSpace/FP/FoundationPose
source /opt/ros/<distro>/setup.bash
source /home/ptcs/wkSpace/FP/dobot/install/setup.bash
python3 run_realsense_yolo_foundationpose.py \
  --dobot_publish_target \
  --dobot_base_T_cam_file /path/to/base_T_cam.txt
```

常用调参：

```bash
--dobot_grasp_offset_obj 0,0.0675,0
--dobot_tcp_to_tip 0,0,0.20
--dobot_grasp_orientation_mode fixed
--dobot_grasp_quat_xyzw qx,qy,qz,qw
--dobot_grasp_object_axis_obj 0,1,0
--dobot_grasp_reference_axis_tcp 1,0,0
--dobot_pose_stable_frames 10
--dobot_pose_max_translation_jitter 0.006
```

`--dobot_tcp_to_tip` 表示 TCP 原点到夹爪末端的向量，单位米，并且是在 TCP 坐标系下表达。视觉端会把它从抓取点里扣掉：让夹爪末端对准瓶子，而不是让 TCP 原点撞到瓶子/桌面。若夹爪还是够不到瓶子，减小这个值；若 TCP 过于靠近瓶子，增大这个值。若夹爪实际沿 TCP 的反方向伸出，把默认值改成 `0,0,-0.15`。

当前默认瓶子模型 `bottle_cad2.obj` 的高度轴是物体坐标系 `Y`，几何中心约在 `Y=0.0675m`，所以默认 `--dobot_grasp_offset_obj 0,0.0675,0` 会把抓取点从瓶底附近移动到瓶身中心附近。

默认使用 `--dobot_grasp_orientation_mode fixed`：优先使用已验证过的固定 TCP 抓取姿态，避免视觉估计出的瓶身方向把腕部转到 Dobot 控制器拒绝的角度。若后续确认控制器能接受随瓶身方向变化的腕部姿态，再临时改成 `planar`。`planar` 会保持固定下探轴，只绕该轴旋转夹爪，把 `--dobot_grasp_reference_axis_tcp` 指定的 TCP 局部轴对齐到 `--dobot_grasp_object_axis_obj` 指定的瓶身长轴投影方向；若发现夹爪开口方向和瓶身差 90 度，把 `--dobot_grasp_reference_axis_tcp 1,0,0` 改成 `0,1,0`。

堆叠或倾斜瓶子需要 3D 姿态时，视觉端使用：

```bash
--dobot_grasp_orientation_mode 3d
```

`3d` 模式会让夹爪参考轴对齐瓶子的 3D 长轴，并自动选择一个垂直于瓶身、且尽量接近固定抓取姿态的接近方向。默认可以直接抓取，不需要预抓取点；如果后续现场确实需要避障，再在 ROS 运控端单独设置 `pre_grasp_height`。

如果只是桌面测试 topic 连通性，可以临时加 `--dobot_allow_identity_handeye`；真实机械臂抓取不要用单位外参。

## SDFR 位姿精修

SDFR 已接在 FoundationPose 后面作为可选精修阶段。开启后，流程变为：

```text
RealSense RGB-D -> YOLO mask -> FoundationPose register/track -> SDFR refine -> 可视化/保存/ROS发布
```

SDFR 精修的是同一份 `ob_in_cam` 位姿，不会把抓取点 offset 叠加进去。抓取点最好在机械臂端根据手眼外参和夹爪策略单独计算，避免破坏视觉位姿的一致性。

运行前需要准备 SDFR 资产：

```text
obj_000001.obj
obj_000001.pth
obj_000001_scale.txt
```

其中 `.pth` 是 SDFR 训练出的隐式曲面模型，`_scale.txt` 会在缺失时自动根据 obj 生成。SDFR 论文/说明里的 obj 默认是毫米单位；实时相机点云和输出位姿仍然使用米。

如果 SDFR 模型 obj 和 FoundationPose 的 `--mesh_file` 不是同一个文件，程序默认会根据两边 OBJ 顶点估计物体坐标系转换。当前瓶子模型就是这种情况：FoundationPose mesh 高度轴是 `Y`，原点接近瓶底；SDFR mesh 高度轴是 `Z`，原点接近瓶身中心。不要直接混用两套坐标系，否则优化后的 bbox 会明显旋转/漂移。

推荐运行方式：

```bash
python run_realsense_yolo_foundationpose.py \
  --use_sdfr \
  --sdfr_model_path /home/ptcs/wkSpace/FP/SDFR/datasets/models/lm/obj_000001.obj \
  --sdfr_register_only 1 \
  --sdfr_refine_interval 1
```

如果 `.pth` 还没训练好，可以离线先按 SDFR 项目的说明训练；不建议实时运行时开启训练。确实要让程序缺失时自动训练，可以加：

```bash
--sdfr_train_if_missing 1
```

运行时会在右侧面板显示一张简短对比表：

```text
        dx   dy   dz   dT   dR
cand   ...  ...  ...  ...  ...
used   ...  ...  ...  ...  ...
```

`cand` 是 SDFR 原始候选结果相对优化前的变化，`used` 是最终实际采用的变化。默认还会保存完整 CSV 到：

```text
debug/realsense_yolo/sdfr_pose_compare.csv
```

如果想专门观察 SDFR 对错误初值的修正能力，可以人为扰动 FoundationPose 给 SDFR 的输入：

```bash
python run_realsense_yolo_foundationpose.py \
  --use_sdfr \
  --sdfr_model_path /home/ptcs/wkSpace/FP/SDFR/datasets/models/lm/obj_000001.obj \
  --sdfr_register_only 0 \
  --sdfr_accept_score_ratio 1.0 \
  --sdfr_test_perturb 1 \
  --sdfr_test_perturb_xyz_mm 5,0,-5
```

这只会扰动送入 SDFR 的 `Before SDFR` 位姿；如果 SDFR 结果被拒绝，主流程会回退到原始 FoundationPose 位姿，避免测试扰动破坏跟踪。

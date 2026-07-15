# dobot_pick_place

这个包是本项目的最小业务控制层，用来把视觉给出的目标位姿转换成机械臂运动，并在到位后控制夹爪。

## 整体数据流

视觉节点或保存的点位发布目标末端位姿：

```text
/dobot_pick/target_pose
geometry_msgs/msg/PoseStamped
```

`target_pose_gripper` 做两类转换：

```text
位置: 米 -> 毫米
姿态: 四元数 -> rx/ry/rz 角度
```

然后调用 Dobot 官方 bringup 提供的服务：

```text
/dobot_bringup_ros2/srv/MovL
dobot_msgs_v4/srv/MovL
```

机械臂接近目标点后，节点发布夹爪闭合控制消息：

```text
/motor_control
step_motor/msg/Motor
```

`target_pose_gripper.launch.py` 默认也会启动当前位姿转换节点和夹爪串口通信节点：

```text
dobot_pick_place/current_pose_publisher
/dobot_msgs_v4/msg/ToolVectorActual -> /dobot_pick/current_pose
```

```text
step_motor/motor_node
usart_port_name=/dev/ttyACM0
serial_baud_rate=115200
```

## 重要细节

Dobot 这个 ROS2 驱动里，`MovL.Request.mode` 的含义如下：

```text
mode=True   -> MovL(joint={j1,j2,j3,j4,j5,j6})
mode=False  -> MovL(pose={x,y,z,rx,ry,rz})
```

视觉给的是末端 TCP 位姿，所以代码里必须使用：

```python
request.mode = False
```

## 记录当前末端位姿

启动当前位姿转换节点：

```bash
ros2 launch dobot_pick_place current_pose_publisher.launch.py
```

查看当前末端位姿：

```bash
ros2 topic echo /dobot_pick/current_pose
```

这个 topic 的格式和 `/dobot_pick/target_pose` 一致，可以把记录下来的位姿直接作为目标点使用。

## 推荐运行顺序

终端 1，启动 Dobot bringup：

```bash
cd ~/wkSpace/dobot
source install/setup.bash
ros2 launch cr_robot_ros2 dobot_bringup_ros2.launch.py
```

终端 2，启动目标位姿控制节点：

```bash
cd ~/wkSpace/dobot
source install/setup.bash
ros2 launch dobot_pick_place target_pose_gripper.launch.py
```

这个 launch 默认会同时启动当前位姿转换节点和夹爪电机串口节点，所以不需要再单独运行：

```bash
ros2 run step_motor motor_node --ros-args -p usart_port_name:=/dev/ttyACM0 -p serial_baud_rate:=115200
```

如果串口号变化，可以这样改：

```bash
ros2 launch dobot_pick_place target_pose_gripper.launch.py gripper_serial_port:=/dev/ttyACM0
```

如果你的夹爪方向接线/安装和默认相反，表现为到位后仍然张开，可以把闭合方向改成 0：

```bash
ros2 launch dobot_pick_place target_pose_gripper.launch.py gripper_close_dir:=0
```

夹爪默认会在末端距离目标点 0.015 m 内闭合。当前闭合速度已经调到 500，所以不需要像之前 0.30 m 那样提前很多闭合；如果现场仍然需要更早或更晚，可以调整：

```bash
ros2 launch dobot_pick_place target_pose_gripper.launch.py gripper_close_distance:=0.02
```

夹爪完全张开指令：

```bash
ros2 topic pub --once /motor_control step_motor/msg/Motor "{id: 1, speed: 500, dir: 0, mode: 2, angle: 9000, state: 0, sub_divide: 32}"
```

夹爪闭合默认发布的消息等价于：

```bash
ros2 topic pub --once /motor_control step_motor/msg/Motor "{id: 1, speed: 500, dir: 1, mode: 2, angle: 40000, state: 0, sub_divide: 32}"
```

`mode=2` 是按角度运行，默认不会再自动发送停止命令，避免夹爪还没闭紧就被 `speed: 0, angle: 0` 打断。

抓取阶段默认不额外修改视觉给出的 z 值。`grasp_z_offset` 只作用于初始抓取点，不影响中间点、箱子点和拍照点。夹爪碰桌面或触发碰撞时，应使用正值把抓取点抬高：

```bash
ros2 launch dobot_pick_place target_pose_gripper.launch.py grasp_z_offset:=0.02
```

如果视觉端使用 `--dobot_grasp_orientation_mode 3d` 做倾斜瓶子或堆叠瓶子的抓取，默认可以直接抓取，不需要预抓取点：

```bash
ros2 launch dobot_pick_place target_pose_gripper.launch.py \
  pre_grasp_height:=0.0
```

如果机械臂已经由 bringup 或示教器使能，但自动初始化服务响应很慢，可以跳过初始化链路，收到视觉目标后直接发运动：

```bash
ros2 launch dobot_pick_place target_pose_gripper.launch.py \
  pre_grasp_height:=0.0 \
  auto_initialize_robot:=false
```

只有在现场确认需要避障时，再打开 TCP 方向预抓取：

```bash
ros2 launch dobot_pick_place target_pose_gripper.launch.py \
  pre_grasp_height:=0.06 \
  pre_grasp_along_tcp:=true \
  pre_grasp_axis_tcp:=0,0,-1
```

`pre_grasp_along_tcp=true` 时，`pre_grasp_height` 不再表示固定向上抬高，而是沿目标 TCP 姿态下的 `pre_grasp_axis_tcp` 反方向后退。

现场测试发现当前夹爪固件可以响应相对角度模式 `mode=2`，但不响应说明书里的绝对角度模式 `mode=4`。因此节点默认使用相对模式张开，并在短时间后自动发布停止命令，避免夹爪到机械限位后持续堵转蜂鸣：

```bash
ros2 launch dobot_pick_place target_pose_gripper.launch.py \
  gripper_open_mode:=2 \
  gripper_open_angle:=9000 \
  gripper_open_duration:=1.0 \
  gripper_prepare_open_before_pick:=true \
  gripper_force_open_before_pick:=true \
  gripper_prepare_delay:=2.0
```

`gripper_open_duration` 控制相对张开命令运行多久后停止；若仍然顶到最大开口蜂鸣，把它调小，例如 `0.6`。`gripper_prepare_delay` 必须大于夹爪实际张开到位的时间。若现场仍看到夹爪还没张开机械臂就开始下探，把它继续调大，例如 `3.0`。

默认抓取流程会直接尝试视觉端给出的抓取 TCP 点，不额外插入预抓取高度，也不再把闭合点抬高。这样能先避免控制器拒绝额外的高位预抓取点。若现场确认上方点可达，再手动开启下面参数做腕部姿态预对齐：

```bash
ros2 launch dobot_pick_place target_pose_gripper.launch.py \
  orientation_align_before_pick:=true \
  orientation_align_angle_threshold:=0.0 \
  orientation_align_pre_grasp_height:=0.08 \
  orientation_align_use_movj:=true \
  orientation_align_fallback_to_movl:=true \
  speed_factor:=60
```

`orientation_align_angle_threshold:=0.0` 表示默认每次抓取都走上方 `MovJ` 预对齐。若控制器拒绝某个 `MovJ` 预对齐点，`orientation_align_fallback_to_movl:=true` 会改用 `MovL` 到同一个上方预抓取点，而不是直接停止流程。若上方空间不够，把 `orientation_align_pre_grasp_height` 调小一些；若现场速度仍偏慢，可以继续提高 `speed_factor`，但要先确认路径周围无遮挡。

如果夹取力度仍然不够，可以继续增大闭合角度；如果夹得过紧，可以调小：

```bash
ros2 launch dobot_pick_place target_pose_gripper.launch.py \
  gripper_close_angle:=38000 \
  gripper_close_duration:=1.2
```

闭合使用相对角度模式夹紧瓶子，但节点会在 `gripper_close_duration` 秒后自动发布停止命令，避免夹爪到机械限位后持续堵转蜂鸣。若仍然蜂鸣，优先减小 `gripper_close_angle` 或缩短 `gripper_close_duration`。

## 抓取后放入箱子

`target_pose_gripper` 现在默认启用抓取后放置流程：

```text
抓取目标点 -> 闭合夹爪 -> 中间点 -> 箱子点释放 -> 保持夹爪开合度回拍照点
```

夹爪只在箱子点发送一次张开命令用于放下瓶子；回拍照点过程中不再额外发送夹爪命令。

当前默认中间点：

```text
-0.6126588745117187,0.09267735290527344,0.06869639587402344,-0.015204590741187188,0.016260905108054526,-0.007558574422311183,0.9997235974699017
```

当前默认箱子释放点：

```text
-0.6518900146484375,0.4343290405273438,0.13612464904785157,-0.034783284399800544,0.0016099064028308628,-0.0031160653596715797,0.9993887239029721
```

当前默认拍照点：

```text
-0.614826416015625,-0.391370849609375,0.004824176788330079,-0.07082562768802304,0.022282501540186947,0.028372318712757964,0.9968361109623641
```

测试阶段的抓取目标点仍由 `send_final_place_pose.launch.py` 发布固定位置。后续接入 foundationpose 时，只需要让 foundationpose 发布同样格式的 `/dobot_pick/target_pose`，就能替代这个固定点。

现在 `target_pose_gripper` 还会发布流程状态，给视觉端做闭环节拍控制：

```text
/dobot_pick/sequence_state
std_msgs/msg/String
```

典型状态变化：

```text
idle -> picking -> to_intermediate -> to_place -> to_camera -> idle
```

FoundationPose 视觉端可以订阅这个状态，只在 `idle` 时发布下一次 `/dobot_pick/target_pose`，这样机械臂完成“抓取 -> 放置 -> 回拍照点”后才会开始下一轮识别抓取。

所以正式运行时仍然按推荐顺序启动即可：

```bash
ros2 launch dobot_pick_place target_pose_gripper.launch.py
```

## FoundationPose 实时传入位姿启动流程

这套流程的数据链路是：

```text
RealSense + YOLO + FoundationPose
  -> UDP 127.0.0.1:5005
  -> foundationpose_udp_bridge
  -> /dobot_pick/target_pose
  -> target_pose_gripper
  -> Dobot MovL + 夹爪
```

先构建 ROS 侧控制包。每次改过 `dobot_pick_place` 代码或 launch 后都需要重新 build：

```bash
cd /home/ptcs/wkSpace/FP/dobot
colcon build --packages-select dobot_pick_place
source install/setup.bash
```

终端 1，启动 Dobot 官方 bringup：

```bash
cd /home/ptcs/wkSpace/FP/dobot
source install/setup.bash
ros2 launch cr_robot_ros2 dobot_bringup_ros2.launch.py
```

终端 2，启动抓取控制、UDP bridge、当前位姿转换和夹爪串口节点：

```bash
cd /home/ptcs/wkSpace/FP/dobot
source install/setup.bash
ros2 launch dobot_pick_place target_pose_gripper.launch.py
```

默认启动时会向夹爪发送一次打开命令，打开角度使用程序默认值 `9000`。如果现场不希望启动时动作，可以加：

```bash
ros2 launch dobot_pick_place target_pose_gripper.launch.py gripper_open_on_start:=false
```

终端 3，启动 FoundationPose，把稳定后的抓取目标通过 UDP 发给 ROS 侧：

```bash
cd /home/ptcs/wkSpace/FP/FoundationPose
conda activate foundationpose
python run_realsense_yolo_foundationpose.py \
  --dobot_publish_target \
  --dobot_eye_in_hand 1
```

当前默认使用眼在手上模式，手眼外参来自 `run_realsense_yolo_foundationpose.py` 的参数默认值：

```text
--dobot_R_cam2gripper
--dobot_t_cam2gripper
```

如果相机是固定在外部支架上，不在末端，需要改成固定相机模式，并提供 `base_T_cam`：

```bash
python run_realsense_yolo_foundationpose.py \
  --dobot_publish_target \
  --dobot_eye_in_hand 0 \
  --dobot_base_T_cam_file /path/to/base_T_cam.txt
```

启动后可以用这些命令检查链路：

```bash
ros2 topic echo /dobot_pick/sequence_state --once
ros2 topic echo /dobot_pick/current_pose --once
ros2 topic echo /dobot_pick/target_pose --once
```

看到 `/dobot_pick/target_pose` 输出后，说明 FoundationPose 已经把目标送到 ROS 侧；`target_pose_gripper` 会在 `sequence_state=idle` 时接收目标并开始抓取。

常用现场参数：

```bash
ros2 launch dobot_pick_place target_pose_gripper.launch.py \
  grasp_z_offset:=0.02 \
  gripper_close_distance:=0.03 \
  gripper_close_angle:=40000
```

如果抓取阶段出现 `res=-50001` 或机械臂碰撞停摆，通常说明视觉目标点太低或路径不可达。先把 `grasp_z_offset` 调成正值，让抓取点整体抬高，例如 `grasp_z_offset:=0.02`。

如果视觉端一直提示 `robot state is unknown` 或 `wait for idle`，检查终端 2 是否启动了 `foundationpose_udp_bridge`，以及 UDP 端口是否保持默认：

```text
ROS 侧接收目标: 127.0.0.1:5005
FoundationPose 侧接收状态: 127.0.0.1:5006
```

如果以后要重新记录中间点或箱子点，它们都使用 `/dobot_pick/current_pose` 这种 PoseStamped 位姿格式。先启动 Dobot bringup 和当前位姿转换节点：

```bash
cd ~/wkSpace/dobot
source install/setup.bash
ros2 launch cr_robot_ros2 dobot_bringup_ros2.launch.py
```

另开终端：

```bash
cd ~/wkSpace/dobot
source install/setup.bash
ros2 launch dobot_pick_place current_pose_publisher.launch.py
```

把机械臂手动拖到中间点后记录当前位姿：

```bash
ros2 topic echo /dobot_pick/current_pose --once
```

从输出中按这个顺序取 7 个数字：

```text
x,y,z,orientation.x,orientation.y,orientation.z,orientation.w
```

例如新的中间点记录成：

```text
intermediate_pose:="-0.500,-0.200,0.420,0.0,0.0,0.0,1.0"
```

再把机械臂拖到箱子释放点，同样记录成：

```text
place_pose:="-0.350,-0.450,0.300,0.0,0.0,0.0,1.0"
```

要临时覆盖默认点位时，在终端 2 启动：

```bash
ros2 launch dobot_pick_place target_pose_gripper.launch.py \
  place_after_grasp:=true \
  intermediate_pose:="-0.500,-0.200,0.420,0.0,0.0,0.0,1.0" \
  place_pose:="-0.350,-0.450,0.300,0.0,0.0,0.0,1.0" \
  camera_pose:="-0.600,-0.390,0.010,0.0,0.0,0.0,1.0"
```

如果机械臂到中间点或箱子点后没有继续下一步，可以适当放大到位阈值：

```bash
ros2 launch dobot_pick_place target_pose_gripper.launch.py transfer_arrival_distance:=0.05
```

如果你想手动启动夹爪节点，可以关闭自动启动：

```bash
ros2 launch dobot_pick_place target_pose_gripper.launch.py start_gripper_motor:=false
```

终端 3，发布保存好的目标点：

```bash
cd ~/wkSpace/dobot
source install/setup.bash
ros2 launch dobot_pick_place send_final_place_pose.launch.py
```

运动前可以检查机械臂状态：

```bash
ros2 topic echo /dobot_msgs_v4/msg/RobotStatus --once
ros2 topic echo /dobot_msgs_v4/msg/ToolVectorActual --once
```

如果网络和机械臂反馈正常，`ToolVectorActual` 不应该全是 0。

如果 `MovL 返回: res=0` 但机械臂不动，先确认 Dobot bringup 终端是否有报警日志，然后手动调用一次官方示例风格的 MovL：

```bash
ros2 service call /dobot_bringup_ros2/srv/MovL dobot_msgs_v4/srv/MovL "{mode: false, a: -638.192, b: -304.897, c: 348.569, d: 1.313, e: 0.146, f: 3.930, param_value: ['user=0', 'tool=0', 'a=50', 'v=80', 'cp=0']}"
```

调用前后分别查看实际 TCP：

```bash
ros2 topic echo /dobot_msgs_v4/msg/ToolVectorActual --once
```

如果手动 MovL 也不动，问题通常在机械臂控制器状态，例如暂停、拖拽模式、报警、急停或队列未运行，而不是 `dobot_pick_place` 节点。


## 控制夹爪
```
开：
ros2 topic pub --once /motor_control step_motor/msg/Motor "{id: 1, speed: 500, dir: 0, mode: 2, angle: 9000, state: 0, sub_divide: 32}"
闭：
ros2 topic pub --once /motor_control step_motor/msg/Motor "{id: 1, speed: 500, dir: 1, mode: 2, angle: 40000, state: 0, sub_divide: 32}"
```

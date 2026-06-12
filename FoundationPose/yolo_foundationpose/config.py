import argparse
import os


code_dir = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))


def parse_args():
  parser = argparse.ArgumentParser()

  parser.add_argument('--mesh_file', type=str, default=f'{code_dir}/yolo_foundationpose/assets/mesh/bottle_cad2.obj',
                      help='目标物体的 3D mesh 文件，FoundationPose 用它渲染候选位姿并估计 6D pose。')
  parser.add_argument('--yolo_weights', type=str, default=f'{code_dir}/yolo_foundationpose/assets/weights/best_1.pt',
                      help='YOLO 分割模型权重路径，用于从 RealSense 彩色图中分割目标。')
  parser.add_argument('--target_cls_id', type=int, default=None,
                      help='只选择指定 YOLO 类别 id；None 表示不限制类别。')

  parser.add_argument('--width', type=int, default=848,
                      help='RealSense 彩色流和深度流宽度。')
  parser.add_argument('--height', type=int, default=480,
                      help='RealSense 彩色流和深度流高度。')
  parser.add_argument('--fps', type=int, default=30,
                      help='RealSense 采集帧率。')
  parser.add_argument('--imgsz', type=int, default=640,
                      help='YOLO 推理输入尺寸，越大通常越准但越慢。')
  parser.add_argument('--conf', type=float, default=0.35,
                      help='YOLO 置信度阈值，低于该值的检测结果会被过滤。')
  parser.add_argument('--yolo_device', type=str, default='0',
                      help='YOLO 推理设备，例如 0 表示第一张 GPU，cpu 表示 CPU。')

  parser.add_argument('--zmin', type=float, default=0.10,
                      help='有效深度最小值，单位米；小于该值的深度会置 0。')
  parser.add_argument('--zmax', type=float, default=2.00,
                      help='有效深度最大值，单位米；大于该值的深度会置 0。')
  parser.add_argument('--min_mask_area', type=int, default=1000,
                      help='YOLO 候选目标的最小 mask 面积，太小的分割会被忽略。')
  parser.add_argument('--min_pose_mask_area', type=int, default=400,
                      help='送入 FoundationPose 前，精修后 mask 的最小面积。')

  parser.add_argument('--est_refine_iter', type=int, default=5,
                      help='FoundationPose register 初始化阶段的 refinement 迭代次数。')
  parser.add_argument('--track_refine_iter', type=int, default=1,
                      help='FoundationPose track_one 跟踪阶段的 refinement 迭代次数。')
  parser.add_argument('--register_every_frame', action='store_true',
                      help='每帧都重新 register；调试可用，但速度明显慢于 track_one。')
  parser.add_argument('--use_sdfr', action='store_true',
                      help='启用 SDFR 姿态精修；开启后在 FoundationPose 后使用 SDFR 优化姿态。')
  parser.add_argument('--sdfr_root', type=str, default=f'{os.path.dirname(code_dir)}/SDFR',
                      help='SDFR 工程根目录；默认查找 FoundationPose 同级目录下的 SDFR。')
  parser.add_argument('--sdfr_model_path', type=str, default=None,
                      help='显式指定 SDFR 使用的 .obj 或 .pth 模型路径；通常使用毫米单位 obj 训练出的 SDFR 模型。')
  parser.add_argument('--sdfr_gpu', type=int, default=0,
                      help='SDFR 使用的 CUDA GPU id。')
  parser.add_argument('--sdfr_max_points', type=int, default=2500,
                      help='SDFR 精修时最多使用多少个 mask 深度点。')
  parser.add_argument('--sdfr_sample_farthest', type=int, default=1,
                      help='SDFR 优化前是否对观测点做 farthest point sampling；1 开启，0 关闭。')
  parser.add_argument('--sdfr_refine_interval', type=int, default=1,
                      help='每隔多少帧运行一次 SDFR 精修；1 表示每帧都跑。')
  parser.add_argument('--sdfr_register_only', type=int, default=1,
                      help='是否只在 register 之后运行 SDFR；1 更稳，0 表示跟踪帧也可运行。')
  parser.add_argument('--sdfr_train_if_missing', type=int, default=0,
                      help='SDFR .pth 不存在时是否在线训练；1 会很慢，实时抓取建议先离线训练好。')
  parser.add_argument('--sdfr_prefer_init_translation', type=int, default=1,
                      help='SDFR 平移优化是否从 FoundationPose 初值开始；1 推荐，0 使用观测点均值。')
  parser.add_argument('--sdfr_use_mesh_alignment', type=int, default=1,
                      help='SDFR 模型和 FoundationPose mesh 路径不同时，是否自动估计二者物体坐标系转换；1 推荐。')
  parser.add_argument('--sdfr_optimize_rotation', type=int, default=0,
                      help='SDFR 是否优化旋转；瓶子等近似旋转对称物体建议先保持 0，只微调平移。')
  parser.add_argument('--sdfr_rotation_regularization', type=float, default=0.0,
                      help='SDFR 旋转正则权重，仅在 --sdfr_optimize_rotation 1 时生效。')
  parser.add_argument('--sdfr_translation_regularization', type=float, default=0.0,
                      help='SDFR 平移正则权重，越大越不容易远离 FoundationPose 初值。')
  parser.add_argument('--sdfr_save_pose_compare', type=int, default=1,
                      help='是否把 SDFR 优化前/候选/实际使用位姿对比保存为 CSV。')
  parser.add_argument('--sdfr_pose_compare_file', type=str, default=None,
                      help='SDFR 位姿对比 CSV 保存路径；默认写入 debug_dir/sdfr_pose_compare.csv。')
  parser.add_argument('--sdfr_test_perturb', type=int, default=0,
                      help='调试用：在送入 SDFR 前人为扰动 FoundationPose 位姿，用于观察 SDFR 能否拉回；0 关闭，1 开启。')
  parser.add_argument('--sdfr_test_perturb_xyz_mm', type=str, default='0,0,0',
                      help='调试扰动平移，单位毫米，格式 x,y,z，例如 5,0,-5。')
  parser.add_argument('--sdfr_test_perturb_rpy_deg', type=str, default='0,0,0',
                      help='调试扰动旋转，单位度，XYZ欧拉角，格式 roll,pitch,yaw，例如 0,0,5。')
  parser.add_argument('--sdfr_refine_scale', type=int, default=0,
                      help='是否同时优化尺度；实时抓取通常关闭。')
  parser.add_argument('--sdfr_initial_scale', type=float, default=0.2,
                      help='开启尺度优化时的初始尺度。')
  parser.add_argument('--sdfr_show_rejected_pose', type=int, default=0,
                      help='调试用：SDFR 候选结果被拒绝时，After SDFR 窗口是否仍显示候选位姿；不会用于跟踪/保存/发布。')
  parser.add_argument('--sdfr_accept_score_ratio', type=float, default=0.98,
                      help='SDFR 结果需要把点云对齐误差至少降到该比例以下才接受。越小越严格。')
  parser.add_argument('--sdfr_accept_max_translation_mm', type=float, default=20.0,
                      help='单次接受的 SDFR 平移改变量上限，单位毫米。')
  parser.add_argument('--sdfr_accept_max_rotation_deg', type=float, default=8.0,
                      help='单次接受的 SDFR 旋转改变量上限，单位度。')
  parser.add_argument('--sdfr_max_iterations', type=int, default=25,
                      help='SDFR 局部优化的最大迭代次数。')
  parser.add_argument('--sdfr_sdf_trunc', type=float, default=0.02,
                      help='SDF 残差截断阈值，单位米。')
  parser.add_argument('--sdfr_depth_band', type=float, default=0.12,
                      help='围绕当前初始位姿深度中心保留观测点的带宽，单位米。')
  parser.add_argument('--sdfr_regularization', type=float, default=0.002,
                      help='SDFR 对位姿增量的正则权重。')
  parser.add_argument('--sdfr_robust_scale', type=float, default=0.01,
                      help='SDFR 鲁棒损失的尺度，单位米。')
  parser.add_argument('--sdfr_translation_limit', type=float, default=0.05,
                      help='SDFR 单次允许的平移修正尺度，单位米。')
  parser.add_argument('--sdfr_rotation_limit_deg', type=float, default=12.0,
                      help='SDFR 单次允许的旋转修正尺度，单位度。')
  parser.add_argument('--sdfr_icp_enable', type=int, default=1,
                      help='SDFR 优化后是否再做一次 ICP 抛光；1 开启，0 关闭。')
  parser.add_argument('--sdfr_icp_max_iterations', type=int, default=15,
                      help='ICP 抛光阶段的最大迭代次数。')
  parser.add_argument('--sdfr_icp_distance_threshold', type=float, default=0.01,
                      help='ICP 对应点最大距离阈值，单位米。')
  parser.add_argument('--sdfr_voxel_size', type=float, default=0.003,
                      help='SDFR/ICP 点云下采样体素大小，单位米。')
  parser.add_argument('--yolo_interval', type=int, default=1,
                      help='每隔多少帧运行一次 YOLO；1 表示每帧运行。')
  parser.add_argument('--register_retry_interval', type=int, default=12,
                      help='两次 register 之间的最小帧间隔，避免失败时每帧重复初始化。')

  parser.add_argument('--mask_open_kernel', type=int, default=3,
                      help='精修 YOLO mask 时开运算核大小，用于去掉小噪点；0 表示关闭。')
  parser.add_argument('--mask_close_kernel', type=int, default=5,
                      help='精修 YOLO mask 时闭运算核大小，用于填补小孔洞；0 表示关闭。')
  parser.add_argument('--mask_erode_kernel', type=int, default=3,
                      help='精修 YOLO mask 时腐蚀核大小，用于收缩边缘，减少背景混入；0 表示关闭。')
  parser.add_argument('--mask_depth_band', type=float, default=0.06,
                      help='精修 mask 时围绕目标深度中位数保留的深度范围，单位米。')
  parser.add_argument('--mask_depth_mad_scale', type=float, default=3.0,
                      help='精修 mask 时基于深度 MAD 的动态阈值倍数，越大保留越宽松。')

  parser.add_argument('--mask_inputs_for_register', type=int, default=1,
                      help='register 时是否把 RGB-D 输入限制在目标 mask 附近；1 开启，0 关闭。')
  parser.add_argument('--mask_inputs_for_track', type=int, default=1,
                      help='track_one 时是否把 RGB-D 输入限制在目标 mask 附近；1 开启，0 关闭。')
  parser.add_argument('--input_mask_dilate_kernel', type=int, default=11,
                      help='限制 FoundationPose 输入前，对目标 mask 膨胀的核大小，避免裁得太紧。')
  parser.add_argument('--input_depth_band', type=float, default=0.10,
                      help='限制 FoundationPose 输入时保留的目标邻近深度范围，单位米。')
  parser.add_argument('--input_depth_mad_scale', type=float, default=4.0,
                      help='限制 FoundationPose 输入时基于深度 MAD 的动态阈值倍数。')

  parser.add_argument('--lock_max_lost', type=int, default=30,
                      help='目标锁定允许连续丢失的最大帧数，超过后释放锁定。')
  parser.add_argument('--lock_switch_after_lost', type=int, default=12,
                      help='目标丢失达到该帧数后，允许切换到新的 YOLO 最优目标。')
  parser.add_argument('--lock_min_iou', type=float, default=0.03,
                      help='YOLO 候选框与当前锁定框匹配所需的最小 IoU。')
  parser.add_argument('--lock_max_center_ratio', type=float, default=0.65,
                      help='目标锁定匹配时允许的最大中心距离比例，越大越容易保持锁定。')
  parser.add_argument('--lock_switch_on_miss', type=int, default=1,
                      help='当前目标漏检时是否允许切换到其他目标；1 允许，0 不允许。')
  parser.add_argument('--follow_yolo_best', type=int, default=0,
                      help='是否始终跟随 YOLO 当前评分最高目标；1 会降低锁定粘性。')
  parser.add_argument('--pose_bbox_slack', type=float, default=1.15,
                      help='检查 pose 与 YOLO 目标是否一致时，投影框允许放大的比例。')
  parser.add_argument('--pose_depth_tolerance', type=float, default=0.18,
                      help='检查 pose 与 YOLO 目标是否一致时允许的深度误差，单位米。')
  parser.add_argument('--pose_min_projected_iou', type=float, default=0.03,
                      help='检查 pose 与 YOLO 目标是否一致时要求的最小投影框 IoU。')
  parser.add_argument('--track_between_yolo', type=int, default=0,
                      help='YOLO 间隔帧是否继续用 FoundationPose 跟踪；1 开启，0 关闭。')

  parser.add_argument('--depth_isolation_weight', type=float, default=0.25,
                      help='目标选择评分中，深度孤立程度的权重。')
  parser.add_argument('--prefer_foreground_target', type=int, default=1,
                      help='多瓶场景下是否优先选择最靠近相机的前景瓶子；1 开启，0 关闭。')
  parser.add_argument('--foreground_depth_window', type=float, default=0.06,
                      help='前景目标深度窗口，单位米；只在最浅候选加该窗口以内的瓶子中按综合评分选择。')
  parser.add_argument('--foreground_switch_depth', type=float, default=0.06,
                      help='锁定深处目标时，若新候选比当前锁定目标近超过该值，则自动切换到前景目标；单位米，0 表示关闭。')
  parser.add_argument('--isolation_ring_kernel', type=int, default=45,
                      help='计算深度孤立程度时，目标周围环形区域的核大小。')
  parser.add_argument('--isolation_depth_band', type=float, default=0.08,
                      help='判断周围干扰物是否接近目标深度的阈值，单位米。')
  parser.add_argument('--max_interference_ratio', type=float, default=0.22,
                      help='允许目标周围存在的最大近深度干扰比例，超过会降低或过滤候选。')

  parser.add_argument('--debug', type=int, default=1,
                      help='FoundationPose 调试等级；越高保存/显示的中间结果越多。')
  parser.add_argument('--debug_dir', type=str, default=f'{code_dir}/debug/realsense_yolo',
                      help='调试输出目录，包括 pose 文本和可选中间可视化结果。')
  parser.add_argument('--save_pose', action='store_true',
                      help='开启后保存每帧 4x4 物体到相机坐标系位姿矩阵。')
  parser.add_argument('--ros_publish_pose', action='store_true',
                      help='开启后通过 ROS 2 topic 发布当前物体到相机坐标系的 PoseStamped。')
  parser.add_argument('--ros_pose_topic', type=str, default='/foundationpose/target_pose',
                      help='ROS 2 位姿发布 topic，消息类型为 geometry_msgs/msg/PoseStamped。')
  parser.add_argument('--ros_pose_frame_id', type=str, default='camera_color_optical_frame',
                      help='PoseStamped.header.frame_id；当前 pose 是物体坐标系相对于该相机光学坐标系的位姿。')
  parser.add_argument('--ros_pose_node_name', type=str, default='foundationpose_pose_publisher',
                      help='ROS 2 发布节点名称。')
  parser.add_argument('--ros_pose_qos_depth', type=int, default=10,
                      help='ROS 2 PoseStamped publisher 的 QoS 队列深度。')

  parser.add_argument('--dobot_publish_target', action='store_true',
                      help='开启后把 FoundationPose 位姿转换成 Dobot 基座下的抓取目标并发布到 /dobot_pick/target_pose。')
  parser.add_argument('--dobot_target_pose_topic', type=str, default='/dobot_pick/target_pose',
                      help='Dobot 抓取节点订阅的目标 TCP PoseStamped topic。')
  parser.add_argument('--dobot_sequence_state_topic', type=str, default='/dobot_pick/sequence_state',
                      help='Dobot 抓取流程状态 topic；视觉端等它回到 idle 后才发布下一次目标。')
  parser.add_argument('--dobot_current_pose_topic', type=str, default='/dobot_pick/current_pose',
                      help='ROS 侧 UDP bridge 读取当前末端 PoseStamped 的 topic。')
  parser.add_argument('--dobot_target_frame_id', type=str, default='dobot_base',
                      help='发布给 Dobot 的 PoseStamped.header.frame_id。')
  parser.add_argument('--dobot_node_name', type=str, default='foundationpose_dobot_target_bridge',
                      help='视觉端 Dobot 目标发布 ROS 2 节点名。')
  parser.add_argument('--dobot_pose_qos_depth', type=int, default=10,
                      help='保留参数；UDP bridge 不直接使用 rclpy QoS。')
  parser.add_argument('--dobot_udp_target_host', type=str, default='127.0.0.1',
                      help='ROS 侧 UDP bridge 接收目标位姿的主机。')
  parser.add_argument('--dobot_udp_target_port', type=int, default=5005,
                      help='ROS 侧 UDP bridge 接收目标位姿的 UDP 端口。')
  parser.add_argument('--dobot_udp_status_bind_host', type=str, default='127.0.0.1',
                      help='FoundationPose 侧接收 Dobot 状态 UDP 的绑定地址。')
  parser.add_argument('--dobot_udp_status_port', type=int, default=5006,
                      help='FoundationPose 侧接收 Dobot 状态 UDP 的端口。')
  parser.add_argument('--dobot_base_T_cam_file', type=str, default='',
                      help='4x4 手眼外参文件，表示相机坐标系到 Dobot 基座坐标系的变换 base_T_cam。')
  parser.add_argument('--dobot_base_T_cam', type=str, default='',
                      help='直接用 16 个逗号分隔数字指定 base_T_cam，行优先排列。')
  parser.add_argument('--dobot_allow_identity_handeye', action='store_true',
                      help='仅用于离线/桌面测试：未提供手眼外参时允许使用单位矩阵。真实机械臂抓取不要开启。')
  parser.add_argument('--dobot_eye_in_hand', type=int, default=1,
                      help='1 表示 D435i 安装在末端，使用当前末端位姿和 cam2gripper 外参动态计算 base_T_cam。')
  parser.add_argument('--dobot_R_cam2gripper', type=str,
                      default=(
                        '0.03201245,0.99392863,0.10526667,'
                        '-0.99936868,0.03345438,-0.01196029,'
                        '-0.01540931,-0.10481733,0.99437210'
                      ),
                      help='眼在手上手眼标定旋转矩阵，表示 p_gripper = R_cam2gripper * p_cam + t。')
  parser.add_argument('--dobot_t_cam2gripper', type=str,
                      default='-0.05104491,0.02979741,0.05619540',
                      help='眼在手上手眼标定平移向量，单位米，表示 p_gripper = R_cam2gripper * p_cam + t。')
  parser.add_argument('--dobot_grasp_offset_obj', type=str, default='0,0.0675,0',
                      help='抓取点相对物体坐标系原点的偏移，单位米，格式 x,y,z；默认移到 bottle_cad2.obj 的几何中心附近。')
  parser.add_argument('--dobot_tcp_to_tip', type=str, default='0,0,0.20',
                      help='TCP 原点到夹爪末端的向量，单位米，在 TCP 坐标系下表达；会从抓取点中扣除该工具长度。')
  parser.add_argument('--dobot_use_object_orientation', type=int, default=0,
                      help='1 表示发布物体姿态作为 TCP 姿态；0 表示使用 dobot_grasp_quat_xyzw 固定抓取姿态。')
  parser.add_argument('--dobot_grasp_orientation_mode', type=str, default='fixed',
                      help='夹爪姿态模式：fixed 使用固定姿态；object 使用完整物体姿态；planar 保持固定下探姿态，仅绕 TCP 轴对齐物体长轴；3d 让夹爪开口轴对齐物体 3D 长轴并自动选择接近方向。')
  parser.add_argument('--dobot_grasp_quat_xyzw', type=str,
                      default='0.011411472351837162,0.0016656594789129023,0.03427464789428714,0.99934591227912',
                      help='固定 TCP 抓取姿态四元数 qx,qy,qz,qw；dobot_use_object_orientation=0 时使用。')
  parser.add_argument('--dobot_grasp_object_axis_obj', type=str, default='0,1,0',
                      help='planar 模式下用于对齐夹爪的物体长轴，物体坐标系表达；bottle_cad2.obj 默认高度/长轴为 Y。')
  parser.add_argument('--dobot_grasp_reference_axis_tcp', type=str, default='1,0,0',
                      help='planar 模式下夹爪希望对齐物体长轴的 TCP 局部轴。若转向相差 90 度，可改成 0,1,0。')
  parser.add_argument('--dobot_wait_for_idle', type=int, default=1,
                      help='是否等待 Dobot sequence_state 属于 idle_states 后才发布目标。')
  parser.add_argument('--dobot_idle_states', type=str, default='idle',
                      help='允许视觉发布下一次目标的 Dobot 状态，逗号分隔。')
  parser.add_argument('--dobot_pose_stable_frames', type=int, default=15,
                      help='目标位置连续稳定多少帧后才发布给机械臂。')
  parser.add_argument('--dobot_pose_max_translation_jitter', type=float, default=0.008,
                      help='稳定窗口内目标位置最大抖动阈值，单位米。')
  parser.add_argument('--dobot_publish_cooldown', type=float, default=2.0,
                      help='两次发布 Dobot 抓取目标之间的最小间隔，单位秒。')
  parser.add_argument('--dobot_min_target_distance', type=float, default=0.05,
                      help='Dobot 基座下目标点距离原点的最小允许距离，单位米，用于过滤明显错误外参/位姿。')
  parser.add_argument('--dobot_max_target_distance', type=float, default=1.20,
                      help='Dobot 基座下目标点距离原点的最大允许距离，单位米，用于过滤明显错误外参/位姿。')
  return parser.parse_args()

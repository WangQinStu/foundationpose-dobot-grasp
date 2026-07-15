#!/usr/bin/env python3
"""接收目标末端位姿，控制 Dobot 机械臂抓取，并可选转运到箱子点释放。

输入:
  /dobot_pick/target_pose  geometry_msgs/msg/PoseStamped

输入位姿使用 ROS 常用表达:
  - xyz: 米
  - orientation: 四元数

Dobot MovL(pose=...) 使用:
  - xyz: 毫米
  - rx/ry/rz: 欧拉角，单位度
"""

import math
import time
from typing import Callable, Optional

import rclpy
from dobot_msgs_v4.msg import RobotStatus
from dobot_msgs_v4.srv import (
    ClearError,
    Continue,
    EnableRobot,
    MovJ,
    MovL,
    PowerOn,
    RequestControl,
    SpeedFactor,
    StopDrag,
)
from geometry_msgs.msg import PoseStamped, Quaternion
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from step_motor.msg import Motor
from std_msgs.msg import String


def quaternion_to_rpy_degrees(q: Quaternion) -> tuple[float, float, float]:
    """将 ROS 四元数转换成 Dobot 需要的 rx/ry/rz 角度。"""
    x = q.x
    y = q.y
    z = q.z
    w = q.w

    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return tuple(math.degrees(v) for v in (roll, pitch, yaw))


class TargetPoseGripper(Node):
    """最小抓取流程节点。

    流程:
      1. 等待 /dobot_pick/target_pose
      2. 首次收到目标时初始化机械臂
      3. 调用 MovL 运动到目标 TCP 位姿
      4. 实时监控当前 TCP 位姿，距离目标小于阈值时闭合夹爪
      5. 可选: 移动到中间点，再移动到箱子点张开夹爪，保持该开合度回到拍照点
    """

    def __init__(self) -> None:
        super().__init__('target_pose_gripper')

        self._declare_parameters()
        self._read_parameters()

        self.request_control_client = self.create_client(RequestControl, self.request_control_service)
        self.power_on_client = self.create_client(PowerOn, self.power_on_service)
        self.stop_drag_client = self.create_client(StopDrag, self.stop_drag_service)
        self.clear_error_client = self.create_client(ClearError, self.clear_error_service)
        self.enable_robot_client = self.create_client(EnableRobot, self.enable_robot_service)
        self.continue_client = self.create_client(Continue, self.continue_service)
        self.speed_factor_client = self.create_client(SpeedFactor, self.speed_factor_service)
        self.movl_client = self.create_client(MovL, self.movl_service)
        self.movj_client = self.create_client(MovJ, self.movj_service)

        self.gripper_pub = self.create_publisher(Motor, self.gripper_topic, 10)
        state_qos = QoSProfile(depth=1)
        state_qos.reliability = ReliabilityPolicy.RELIABLE
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.sequence_state_pub = self.create_publisher(String, self.sequence_state_topic, state_qos)
        self.create_subscription(RobotStatus, self.robot_status_topic, self.robot_status_callback, 10)
        self.create_subscription(PoseStamped, self.target_pose_topic, self.target_pose_callback, 10)
        self.create_subscription(PoseStamped, self.current_pose_topic, self.current_pose_callback, 10)

        self.busy = False
        self.robot_ready = False
        self.robot_is_enable = False
        self.robot_is_connected = False
        self.pending_pick_pose: Optional[PoseStamped] = None
        self.active_target_pose: Optional[PoseStamped] = None
        self.pick_orientation_fallback_used = False
        self.gripper_closed_for_target = False
        self.intermediate_reached = False
        self.place_reached = False
        self.camera_reached = False
        self.motion_stage = 'idle'
        self.connection_wait_started_at = 0.0
        self.connection_wait_timer = None
        self.enable_wait_started_at = 0.0
        self.enable_wait_timer = None
        self.gripper_stop_timer = None
        self.startup_open_timer = None
        self.pick_close_timeout_timer = None
        self.after_grasp_timer = None

        self.get_logger().info(
            f'监听目标 {self.target_pose_topic}，监听当前位姿 {self.current_pose_topic}，'
            f'调用 {self.movl_service}，夹爪发布到 {self.gripper_topic}'
        )
        self.publish_sequence_state()
        if self._parse_bool_parameter(self.get_parameter('gripper_open_on_start').value):
            self.startup_open_timer = self.create_timer(0.5, self.open_gripper_on_start)

    def _declare_parameters(self) -> None:
        self.declare_parameter('target_pose_topic', '/dobot_pick/target_pose')
        self.declare_parameter('current_pose_topic', '/dobot_pick/current_pose')
        self.declare_parameter('robot_status_topic', '/dobot_msgs_v4/msg/RobotStatus')
        self.declare_parameter('gripper_topic', '/motor_control')
        self.declare_parameter('sequence_state_topic', '/dobot_pick/sequence_state')

        self.declare_parameter('request_control_service', '/dobot_bringup_ros2/srv/RequestControl')
        self.declare_parameter('power_on_service', '/dobot_bringup_ros2/srv/PowerOn')
        self.declare_parameter('stop_drag_service', '/dobot_bringup_ros2/srv/StopDrag')
        self.declare_parameter('clear_error_service', '/dobot_bringup_ros2/srv/ClearError')
        self.declare_parameter('enable_robot_service', '/dobot_bringup_ros2/srv/EnableRobot')
        self.declare_parameter('continue_service', '/dobot_bringup_ros2/srv/Continue')
        self.declare_parameter('speed_factor_service', '/dobot_bringup_ros2/srv/SpeedFactor')
        self.declare_parameter('movl_service', '/dobot_bringup_ros2/srv/MovL')
        self.declare_parameter('movj_service', '/dobot_bringup_ros2/srv/MovJ')

        # PoseStamped 的位置单位是米，Dobot MovL(pose=...) 的位置单位是毫米。
        self.declare_parameter('position_scale', 1000.0)
        self.declare_parameter('movl_param_value', ['user=0', 'tool=0', 'a=50', 'v=80', 'cp=0'])
        self.declare_parameter('auto_initialize_robot', True)
        self.declare_parameter('ignore_request_control_failure', True)
        self.declare_parameter('ignore_speed_factor_failure', True)
        self.declare_parameter('fallback_to_movj', True)
        self.declare_parameter('speed_factor', 30)
        self.declare_parameter('wait_for_robot_connection', True)
        self.declare_parameter('connection_wait_timeout', 15.0)
        self.declare_parameter('trust_enable_service_response', True)
        self.declare_parameter('enable_wait_timeout', 8.0)
        self.declare_parameter('grasp_z_offset', 0.0)
        self.declare_parameter('pre_grasp_height', 0.0)
        self.declare_parameter('pre_grasp_along_tcp', True)
        self.declare_parameter('pre_grasp_axis_tcp', '0,0,-1')
        self.declare_parameter('fallback_to_pick_when_pre_grasp_rejected', True)
        self.declare_parameter('fallback_pick_orientation_on_reject', True)
        self.declare_parameter(
            'fallback_pick_quat_xyzw',
            '0.011411472351837162,0.0016656594789129023,0.03427464789428714,0.99934591227912',
        )
        self.declare_parameter('gripper_close_distance', 0.008)
        self.declare_parameter('place_after_grasp', False)
        self.declare_parameter('intermediate_pose', '')
        self.declare_parameter('place_pose', '')
        self.declare_parameter('return_to_camera_after_place', True)
        self.declare_parameter('camera_pose', '')
        self.declare_parameter('transfer_arrival_distance', 0.03)
        self.declare_parameter('after_grasp_delay', 0.8)

        # WHEELTEC 步进电机夹爪参数。mode=2 为位置/角度模式，默认只发布目标角度命令。
        self.declare_parameter('gripper_id', 1)
        self.declare_parameter('gripper_open_dir', 0)
        self.declare_parameter('gripper_open_mode', 2)
        self.declare_parameter('gripper_open_speed', 500)
        self.declare_parameter('gripper_open_angle', 9000)
        self.declare_parameter('gripper_close_dir', 1)
        self.declare_parameter('gripper_close_mode', 2)
        self.declare_parameter('gripper_close_speed', 500)
        self.declare_parameter('gripper_close_angle', 40000)
        self.declare_parameter('gripper_sub_divide', 32)
        self.declare_parameter('gripper_close_duration', 0.0)
        self.declare_parameter('gripper_open_on_start', True)
        self.declare_parameter('pick_close_timeout', 0.0)

    def _read_parameters(self) -> None:
        self.target_pose_topic = self.get_parameter('target_pose_topic').value
        self.current_pose_topic = self.get_parameter('current_pose_topic').value
        self.robot_status_topic = self.get_parameter('robot_status_topic').value
        self.gripper_topic = self.get_parameter('gripper_topic').value
        self.sequence_state_topic = self.get_parameter('sequence_state_topic').value

        self.request_control_service = self.get_parameter('request_control_service').value
        self.power_on_service = self.get_parameter('power_on_service').value
        self.stop_drag_service = self.get_parameter('stop_drag_service').value
        self.clear_error_service = self.get_parameter('clear_error_service').value
        self.enable_robot_service = self.get_parameter('enable_robot_service').value
        self.continue_service = self.get_parameter('continue_service').value
        self.speed_factor_service = self.get_parameter('speed_factor_service').value
        self.movl_service = self.get_parameter('movl_service').value
        self.movj_service = self.get_parameter('movj_service').value

        self.position_scale = float(self.get_parameter('position_scale').value)
        self.movl_param_value = list(self.get_parameter('movl_param_value').value)
        self.auto_initialize_robot = bool(self.get_parameter('auto_initialize_robot').value)
        self.wait_for_robot_connection = self._parse_bool_parameter(
            self.get_parameter('wait_for_robot_connection').value
        )
        self.connection_wait_timeout = float(self.get_parameter('connection_wait_timeout').value)
        self.trust_enable_service_response = self._parse_bool_parameter(
            self.get_parameter('trust_enable_service_response').value
        )
        self.enable_wait_timeout = float(self.get_parameter('enable_wait_timeout').value)
        self.grasp_z_offset = float(self.get_parameter('grasp_z_offset').value)
        self.pre_grasp_height = float(self.get_parameter('pre_grasp_height').value)
        self.pre_grasp_along_tcp = self._parse_bool_parameter(self.get_parameter('pre_grasp_along_tcp').value)
        self.pre_grasp_axis_tcp = self._parse_vector_parameter(
            str(self.get_parameter('pre_grasp_axis_tcp').value),
            'pre_grasp_axis_tcp',
        )
        self.fallback_to_pick_when_pre_grasp_rejected = self._parse_bool_parameter(
            self.get_parameter('fallback_to_pick_when_pre_grasp_rejected').value
        )
        self.fallback_pick_orientation_on_reject = self._parse_bool_parameter(
            self.get_parameter('fallback_pick_orientation_on_reject').value
        )
        self.fallback_pick_quat_xyzw = self._parse_vector4_parameter(
            str(self.get_parameter('fallback_pick_quat_xyzw').value),
            'fallback_pick_quat_xyzw',
        )
        self.gripper_close_distance = float(self.get_parameter('gripper_close_distance').value)
        self.place_after_grasp = self._parse_bool_parameter(self.get_parameter('place_after_grasp').value)
        self.transfer_arrival_distance = float(self.get_parameter('transfer_arrival_distance').value)
        self.after_grasp_delay = float(self.get_parameter('after_grasp_delay').value)
        self.intermediate_pose = self._parse_pose_parameter(
            str(self.get_parameter('intermediate_pose').value),
            'intermediate_pose',
        )
        self.place_pose = self._parse_pose_parameter(
            str(self.get_parameter('place_pose').value),
            'place_pose',
        )
        self.return_to_camera_after_place = self._parse_bool_parameter(
            self.get_parameter('return_to_camera_after_place').value
        )
        self.camera_pose = self._parse_pose_parameter(
            str(self.get_parameter('camera_pose').value),
            'camera_pose',
        )

    def robot_status_callback(self, msg: RobotStatus) -> None:
        # 不直接相信 service 返回值；以实时状态 topic 判断机械臂是否真的使能。
        was_connected = self.robot_is_connected
        self.robot_is_enable = bool(msg.is_enable)
        self.robot_is_connected = bool(msg.is_connected)
        if self.robot_is_connected and not was_connected:
            self.get_logger().info('RobotStatus 显示机械臂 TCP 已连接。')
            if self.busy and self.pending_pick_pose is not None and self.connection_wait_timer is not None:
                self.cancel_connection_wait()
                self.start_pending_motion()
        elif was_connected and not self.robot_is_connected:
            self.robot_ready = False
            self.get_logger().error('机械臂 TCP 连接已断开，后续运动指令将停止发送。')
            if self.busy:
                self.finish_motion('抓取过程中机械臂 TCP 连接断开，已停止当前流程。')

    def current_pose_callback(self, msg: PoseStamped) -> None:
        if self.active_target_pose is None:
            return

        distance = self.position_distance(msg, self.active_target_pose)
        if self.motion_stage == 'picking':
            if self.gripper_closed_for_target or distance > self.gripper_close_distance:
                return

            self.gripper_closed_for_target = True
            self.cancel_pick_close_timeout()
            self.get_logger().info(
                f'末端距离抓取目标 {distance:.3f} m，小于阈值 {self.gripper_close_distance:.3f} m，闭合夹爪。'
            )
            self.publish_gripper_close()
            self.after_grasp_action()
            return

        if self.motion_stage == 'to_pre_grasp':
            if distance > self.transfer_arrival_distance:
                return

            self.get_logger().info(
                f'已到达预抓取点附近，距离 {distance:.3f} m，下降到抓取点。'
            )
            self.send_pose_movl(self.pending_pick_pose, '抓取点', 'picking')
            return

        if self.motion_stage == 'to_intermediate':
            if self.intermediate_reached or distance > self.transfer_arrival_distance:
                return

            self.intermediate_reached = True
            self.get_logger().info(f'已到达中间点附近，距离 {distance:.3f} m，继续移动到箱子点。')
            self.send_pose_movl(self.place_pose, '箱子点', 'to_place')
            return

        if self.motion_stage == 'to_place':
            if self.place_reached or distance > self.transfer_arrival_distance:
                return

            self.place_reached = True
            self.get_logger().info(
                f'已到达箱子点附近，距离 {distance:.3f} m，张开夹爪释放。'
            )
            self.publish_gripper_open()
            if self.return_to_camera_after_place:
                self.get_logger().info('保持当前夹爪开合度，移动回拍照点。')
                self.send_pose_movl(self.camera_pose, '拍照点', 'to_camera')
                return
            self.finish_sequence()
            return

        if self.motion_stage == 'to_camera':
            if self.camera_reached or distance > self.transfer_arrival_distance:
                return

            self.camera_reached = True
            self.get_logger().info(
                f'已回到拍照点附近，距离 {distance:.3f} m，等待下一次目标位姿。'
            )
            self.finish_sequence()

    def target_pose_callback(self, msg: PoseStamped) -> None:
        if self.busy:
            self.get_logger().warn('上一条运动还没结束，忽略新的目标位姿。')
            return

        if not self._wait_for_service(self.movl_client, self.movl_service):
            return

        self.busy = True
        self.set_motion_stage('picking')
        pick_pose = self._copy_pose_with_offset(msg, z_offset=self.grasp_z_offset)
        self.active_target_pose = pick_pose
        self.gripper_closed_for_target = False
        self.pick_orientation_fallback_used = False
        self.intermediate_reached = False
        self.place_reached = False
        self.camera_reached = False
        self.pending_pick_pose = pick_pose
        if abs(self.grasp_z_offset) > 0.0:
            self.get_logger().info(
                f'抓取目标 z 偏移 {self.grasp_z_offset:.3f} m，'
                f'实际抓取 z={pick_pose.pose.position.z:.3f} m。'
            )
        else:
            self.get_logger().info(
                f'抓取目标 z={pick_pose.pose.position.z:.3f} m。'
            )

        if self.place_after_grasp and (self.intermediate_pose is None or self.place_pose is None):
            self.finish_motion('place_after_grasp=true，但 intermediate_pose 或 place_pose 为空。')
            return
        if self.place_after_grasp and self.return_to_camera_after_place and self.camera_pose is None:
            self.finish_motion('return_to_camera_after_place=true，但 camera_pose 为空。')
            return

        if not self.robot_is_connected:
            if not self.wait_for_robot_connection:
                self.finish_motion(
                    '机械臂 TCP 未连接，拒绝发送运动指令。请检查 IP、网线和 RobotStatus.is_connected。'
                )
                return
            self.connection_wait_started_at = time.monotonic()
            self.connection_wait_timer = self.create_timer(0.2, self.check_robot_connection)
            self.get_logger().warn(
                f'机械臂 TCP 尚未连接，最多等待 {self.connection_wait_timeout:.1f} 秒；'
                '连接成功后再执行抓取。'
            )
            return

        self.start_pending_motion()

    def start_pending_motion(self) -> None:
        if not self.busy or self.pending_pick_pose is None:
            return
        if not self.robot_is_connected:
            self.finish_motion(
                '机械臂 TCP 未连接，拒绝发送运动指令。请检查 IP、网线和 RobotStatus.is_connected。'
            )
            return
        if self.auto_initialize_robot and not self.robot_ready:
            self.initialize_robot()
        else:
            self.send_pending_movl()

    def check_robot_connection(self) -> None:
        if self.robot_is_connected:
            self.cancel_connection_wait()
            self.start_pending_motion()
            return

        if time.monotonic() - self.connection_wait_started_at >= self.connection_wait_timeout:
            self.cancel_connection_wait()
            self.finish_motion(
                '等待机械臂 TCP 连接超时。当前控制器不可达，请检查 IP_address、'
                '电脑有线网卡地址、网线和控制器电源。'
            )

    def cancel_connection_wait(self) -> None:
        if self.connection_wait_timer is not None:
            self.connection_wait_timer.cancel()
            self.connection_wait_timer = None

    def initialize_robot(self) -> None:
        if not self.robot_is_connected:
            self.robot_ready = False
            self.finish_motion('机械臂 TCP 未连接，不能执行 RequestControl。')
            return
        if self.robot_is_connected and self.robot_is_enable:
            self.get_logger().info('RobotStatus 已连接且已使能，跳过 RequestControl/PowerOn/EnableRobot，直接设置速度。')
            self.send_speed_factor()
            return

        self.get_logger().info(
            '初始化机械臂: RequestControl -> PowerOn -> StopDrag -> ClearError -> '
            'EnableRobot -> Continue(可选) -> SpeedFactor'
        )
        self._call_simple_service(
            self.request_control_client,
            RequestControl.Request(),
            'RequestControl',
            self.on_request_control_done,
        )

    def on_request_control_done(self, ok: bool) -> None:
        if not ok:
            if not self.robot_is_connected:
                self.robot_ready = False
                self.finish_motion('RequestControl 失败：机械臂 TCP 已断开。')
                return
            if self.robot_is_connected and self.robot_is_enable:
                self.get_logger().warn(
                    'RequestControl 返回失败，但 RobotStatus 显示已连接且已使能，继续设置速度并执行运动。'
                )
                self.send_speed_factor()
                return
            if self._parse_bool_parameter(self.get_parameter('ignore_request_control_failure').value):
                self.get_logger().warn(
                    'RequestControl 返回失败，但 ignore_request_control_failure=true，继续设置速度并执行运动。'
                )
                self.send_speed_factor()
                return
            self.finish_motion('RequestControl 失败，请在示教器/控制器上确认已允许远程控制。')
            return
        self._call_simple_service(self.power_on_client, PowerOn.Request(), 'PowerOn', self.on_power_on_done)

    def on_power_on_done(self, ok: bool) -> None:
        if not ok:
            self.finish_motion('PowerOn 失败。')
            return
        self._call_simple_service(self.stop_drag_client, StopDrag.Request(), 'StopDrag', self.on_stop_drag_done)

    def on_stop_drag_done(self, ok: bool) -> None:
        if not ok:
            self.finish_motion('StopDrag 失败，请确认机械臂是否处于可退出拖拽状态。')
            return
        self._call_simple_service(self.clear_error_client, ClearError.Request(), 'ClearError', self.on_clear_error_done)

    def on_clear_error_done(self, ok: bool) -> None:
        if not ok:
            self.finish_motion('ClearError 失败。')
            return
        self._call_simple_service(self.enable_robot_client, EnableRobot.Request(), 'EnableRobot', self.on_enable_done)

    def on_enable_done(self, ok: bool) -> None:
        if not ok:
            self.finish_motion('EnableRobot 失败。')
            return
        if not self.continue_client.service_is_ready():
            self.get_logger().warn('Continue 服务不可用，跳过 Continue。')
            self.wait_until_enabled()
            return
        self._call_simple_service(self.continue_client, Continue.Request(), 'Continue', self.on_continue_done)

    def on_continue_done(self, ok: bool) -> None:
        if not ok:
            self.finish_motion('Continue 失败，请检查机械臂是否处于暂停、报警或急停状态。')
            return
        self.wait_until_enabled()

    def wait_until_enabled(self) -> None:
        if self.robot_is_enable:
            self.send_speed_factor()
            return

        if self.trust_enable_service_response:
            self.get_logger().warn(
                'EnableRobot/Continue 已返回成功，但 RobotStatus.is_enable 仍为 false；'
                'trust_enable_service_response=true，继续设置速度并执行运动。'
            )
            self.send_speed_factor()
            return

        self.get_logger().warn(
            f'EnableRobot 已返回，但 RobotStatus 仍未使能，等待真实使能状态... '
            f'is_connected={self.robot_is_connected}'
        )
        self.enable_wait_started_at = time.monotonic()
        self.enable_wait_timer = self.create_timer(0.2, self.check_enable_state)

    def check_enable_state(self) -> None:
        if self.robot_is_enable:
            self.enable_wait_timer.cancel()
            self.enable_wait_timer = None
            self.get_logger().info(f'RobotStatus 已使能: is_connected={self.robot_is_connected}')
            self.send_speed_factor()
            return

        elapsed = time.monotonic() - self.enable_wait_started_at
        if elapsed >= self.enable_wait_timeout:
            self.enable_wait_timer.cancel()
            self.enable_wait_timer = None
            self.finish_motion('等待机械臂使能超时，请检查 RobotStatus/is_enable。')

    def send_speed_factor(self) -> None:
        if not self.robot_is_connected:
            self.robot_ready = False
            self.finish_motion('机械臂 TCP 已断开，不能设置 SpeedFactor。')
            return
        request = SpeedFactor.Request()
        request.ratio = int(self.get_parameter('speed_factor').value)
        self._call_simple_service(self.speed_factor_client, request, 'SpeedFactor', self.on_speed_factor_done)

    def on_speed_factor_done(self, ok: bool) -> None:
        if not ok:
            if not self.robot_is_connected:
                self.robot_ready = False
                self.finish_motion('SpeedFactor 失败：机械臂 TCP 已断开。')
                return
            if self._parse_bool_parameter(self.get_parameter('ignore_speed_factor_failure').value):
                self.get_logger().warn(
                    'SpeedFactor 返回失败，但 ignore_speed_factor_failure=true，继续执行运动。'
                )
                self.robot_ready = True
                self.send_pending_movl()
                return
            self.finish_motion('SpeedFactor 失败。')
            return
        self.robot_ready = True
        self.send_pending_movl()

    def send_pending_movl(self) -> None:
        if self.pending_pick_pose is None:
            self.finish_motion('没有待执行的抓取位姿。')
            return

        if self.pre_grasp_height > 0.0:
            if self.pre_grasp_along_tcp:
                pre_grasp_pose = self._copy_pose_back_along_tcp(
                    self.pending_pick_pose,
                    distance=self.pre_grasp_height,
                    axis_tcp=self.pre_grasp_axis_tcp,
                )
            else:
                pre_grasp_pose = self._copy_pose_with_offset(
                    self.pending_pick_pose,
                    z_offset=self.pre_grasp_height,
                )
            self.send_pose_movl(pre_grasp_pose, '预抓取点', 'to_pre_grasp')
            return

        self.send_pose_movl(self.pending_pick_pose, '抓取点', 'picking')

    @staticmethod
    def position_distance(a: PoseStamped, b: PoseStamped) -> float:
        dx = a.pose.position.x - b.pose.position.x
        dy = a.pose.position.y - b.pose.position.y
        dz = a.pose.position.z - b.pose.position.z
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def _pose_to_movl_request(self, msg: PoseStamped) -> MovL.Request:
        pose = msg.pose
        rx, ry, rz = quaternion_to_rpy_degrees(pose.orientation)

        request = MovL.Request()

        # Dobot 驱动里的 mode 含义:
        #   True  -> MovL(joint={j1,j2,j3,j4,j5,j6})
        #   False -> MovL(pose={x,y,z,rx,ry,rz})
        # 视觉给的是 TCP 位姿，所以必须使用 False。
        request.mode = False
        request.a = float(pose.position.x) * self.position_scale
        request.b = float(pose.position.y) * self.position_scale
        request.c = float(pose.position.z) * self.position_scale
        request.d = rx
        request.e = ry
        request.f = rz
        request.param_value = self.movl_param_value
        return request

    def _pose_to_movj_request(self, msg: PoseStamped) -> MovJ.Request:
        movl_request = self._pose_to_movl_request(msg)
        request = MovJ.Request()
        request.mode = movl_request.mode
        request.a = movl_request.a
        request.b = movl_request.b
        request.c = movl_request.c
        request.d = movl_request.d
        request.e = movl_request.e
        request.f = movl_request.f
        request.param_value = movl_request.param_value
        return request

    @staticmethod
    def _copy_pose_with_offset(msg: PoseStamped, z_offset: float = 0.0) -> PoseStamped:
        copied = PoseStamped()
        copied.header = msg.header
        copied.pose.position.x = msg.pose.position.x
        copied.pose.position.y = msg.pose.position.y
        copied.pose.position.z = msg.pose.position.z + z_offset
        copied.pose.orientation = msg.pose.orientation
        return copied

    @staticmethod
    def _copy_pose_with_orientation(
        msg: PoseStamped,
        quat_xyzw: tuple[float, float, float, float],
    ) -> PoseStamped:
        copied = PoseStamped()
        copied.header = msg.header
        copied.pose.position.x = msg.pose.position.x
        copied.pose.position.y = msg.pose.position.y
        copied.pose.position.z = msg.pose.position.z
        copied.pose.orientation.x = quat_xyzw[0]
        copied.pose.orientation.y = quat_xyzw[1]
        copied.pose.orientation.z = quat_xyzw[2]
        copied.pose.orientation.w = quat_xyzw[3]
        return copied

    @classmethod
    def _copy_pose_back_along_tcp(
        cls,
        msg: PoseStamped,
        distance: float,
        axis_tcp: tuple[float, float, float],
    ) -> PoseStamped:
        axis_base = cls._rotate_vector_by_quaternion(axis_tcp, msg.pose.orientation)
        norm = math.sqrt(sum(component * component for component in axis_base))
        if norm < 1e-9:
            axis_base = (0.0, 0.0, 1.0)
            norm = 1.0
        axis_base = tuple(component / norm for component in axis_base)

        copied = PoseStamped()
        copied.header = msg.header
        copied.pose.position.x = msg.pose.position.x - axis_base[0] * distance
        copied.pose.position.y = msg.pose.position.y - axis_base[1] * distance
        copied.pose.position.z = msg.pose.position.z - axis_base[2] * distance
        copied.pose.orientation = msg.pose.orientation
        return copied

    @staticmethod
    def _rotate_vector_by_quaternion(
        vector: tuple[float, float, float],
        q: Quaternion,
    ) -> tuple[float, float, float]:
        x, y, z = vector
        qx = q.x
        qy = q.y
        qz = q.z
        qw = q.w

        # v' = v + 2*qw*(q_xyz x v) + 2*(q_xyz x (q_xyz x v))
        tx = 2.0 * (qy * z - qz * y)
        ty = 2.0 * (qz * x - qx * z)
        tz = 2.0 * (qx * y - qy * x)
        return (
            x + qw * tx + (qy * tz - qz * ty),
            y + qw * ty + (qz * tx - qx * tz),
            z + qw * tz + (qx * ty - qy * tx),
        )

    def publish_gripper_close(self) -> None:
        command = Motor()
        command.id = int(self.get_parameter('gripper_id').value)
        command.dir = int(self.get_parameter('gripper_close_dir').value)
        command.mode = int(self.get_parameter('gripper_close_mode').value)
        command.speed = int(self.get_parameter('gripper_close_speed').value)
        command.angle = int(self.get_parameter('gripper_close_angle').value)
        command.state = 0
        command.sub_divide = int(self.get_parameter('gripper_sub_divide').value)
        self.gripper_pub.publish(command)
        self.get_logger().info('已发布夹爪闭合命令。')

        duration = float(self.get_parameter('gripper_close_duration').value)
        if duration > 0.0:
            self.gripper_stop_timer = self.create_timer(duration, self.publish_gripper_stop)

    def publish_gripper_open(self) -> None:
        command = Motor()
        command.id = int(self.get_parameter('gripper_id').value)
        command.dir = int(self.get_parameter('gripper_open_dir').value)
        command.mode = int(self.get_parameter('gripper_open_mode').value)
        command.speed = int(self.get_parameter('gripper_open_speed').value)
        command.angle = int(self.get_parameter('gripper_open_angle').value)
        command.state = 0
        command.sub_divide = int(self.get_parameter('gripper_sub_divide').value)
        self.gripper_pub.publish(command)
        self.get_logger().info('已发布夹爪张开命令。')

    def open_gripper_on_start(self) -> None:
        if self.startup_open_timer is not None:
            self.startup_open_timer.cancel()
            self.startup_open_timer = None
        self.publish_gripper_open()

    def start_pick_close_timeout(self) -> None:
        self.cancel_pick_close_timeout()
        timeout = float(self.get_parameter('pick_close_timeout').value)
        if timeout <= 0.0:
            return
        self.pick_close_timeout_timer = self.create_timer(timeout, self.close_gripper_after_pick_timeout)

    def cancel_pick_close_timeout(self) -> None:
        if self.pick_close_timeout_timer is not None:
            self.pick_close_timeout_timer.cancel()
            self.pick_close_timeout_timer = None

    def close_gripper_after_pick_timeout(self) -> None:
        self.cancel_pick_close_timeout()
        if self.motion_stage != 'picking' or self.gripper_closed_for_target:
            return
        self.gripper_closed_for_target = True
        self.get_logger().warn(
            '等待当前位姿进入夹爪闭合阈值超时，执行兜底闭合夹爪。'
        )
        self.publish_gripper_close()
        self.after_grasp_action()

    def after_grasp_action(self) -> None:
        if not self.place_after_grasp:
            self.finish_sequence()
            return

        if self.after_grasp_delay <= 0.0:
            self.send_pose_movl(self.intermediate_pose, '中间点', 'to_intermediate')
            return

        self.after_grasp_timer = self.create_timer(self.after_grasp_delay, self.start_transfer_after_grasp)

    def start_transfer_after_grasp(self) -> None:
        if self.after_grasp_timer is not None:
            self.after_grasp_timer.cancel()
            self.after_grasp_timer = None
        self.send_pose_movl(self.intermediate_pose, '中间点', 'to_intermediate')

    def send_pose_movl(self, pose: Optional[PoseStamped], label: str, stage: str) -> None:
        if pose is None:
            self.finish_motion(f'{label}为空，无法继续。')
            return
        if not self.robot_is_connected:
            self.robot_ready = False
            self.finish_motion(f'机械臂 TCP 已断开，未发送 MovL {label}。')
            return

        self.set_motion_stage(stage)
        self.active_target_pose = pose
        request = self._pose_to_movl_request(pose)
        future = self.movl_client.call_async(request)
        future.add_done_callback(
            lambda motion_future: self.on_motion_done(
                motion_future,
                pose,
                label,
                stage,
                'MovL',
                fallback_allowed=True,
            )
        )
        self.get_logger().info(
            f'发送 MovL {label}: '
            f'x={request.a:.3f}, y={request.b:.3f}, z={request.c:.3f}, '
            f'rx={request.d:.3f}, ry={request.e:.3f}, rz={request.f:.3f}'
        )

    def send_pose_movj(self, pose: PoseStamped, label: str, stage: str) -> None:
        if not self.robot_is_connected:
            self.robot_ready = False
            self.finish_motion(f'机械臂 TCP 已断开，未发送 MovJ {label}。')
            return
        if not self._wait_for_service(self.movj_client, self.movj_service):
            self.finish_motion(f'{self.movj_service} 服务不可用，无法执行 MovJ 兜底。')
            return

        request = self._pose_to_movj_request(pose)
        future = self.movj_client.call_async(request)
        future.add_done_callback(
            lambda motion_future: self.on_motion_done(
                motion_future,
                pose,
                label,
                stage,
                'MovJ',
                fallback_allowed=False,
            )
        )
        self.get_logger().info(
            f'发送 MovJ {label}: '
            f'x={request.a:.3f}, y={request.b:.3f}, z={request.c:.3f}, '
            f'rx={request.d:.3f}, ry={request.e:.3f}, rz={request.f:.3f}'
        )

    def on_motion_done(
        self,
        future,
        pose: PoseStamped,
        label: str,
        stage: str,
        motion_name: str,
        fallback_allowed: bool,
    ) -> None:
        try:
            response = future.result()
        except Exception as exc:
            self.finish_motion(f'{motion_name} 调用异常: {exc}')
            return

        if response is None:
            self.finish_motion(f'{motion_name} 没有返回响应。')
            return

        self.get_logger().info(
            f'{motion_name} 返回: res={response.res}, robot_return="{response.robot_return}"'
        )
        if response.res != 0:
            if not self.robot_is_connected:
                self.robot_ready = False
                self.finish_motion(f'{motion_name} {label} 失败：机械臂 TCP 已断开。')
                return
            if (
                fallback_allowed
                and self._parse_bool_parameter(self.get_parameter('fallback_to_movj').value)
            ):
                self.get_logger().warn(
                    f'{motion_name} {label} 被控制器拒绝，改用 MovJ 点到点运动兜底。'
                )
                self.send_pose_movj(pose, label, stage)
                return
            if stage == 'to_pre_grasp' and self.fallback_to_pick_when_pre_grasp_rejected:
                self.get_logger().warn(
                    f'{motion_name} 预抓取点被控制器拒绝，跳过预抓取点，直接尝试抓取点。'
                )
                self.send_pose_movl(self.pending_pick_pose, '抓取点', 'picking')
                return
            if (
                stage == 'picking'
                and self.fallback_pick_orientation_on_reject
                and not self.pick_orientation_fallback_used
            ):
                self.pick_orientation_fallback_used = True
                fallback_pose = self._copy_pose_with_orientation(pose, self.fallback_pick_quat_xyzw)
                self.pending_pick_pose = fallback_pose
                self.active_target_pose = fallback_pose
                self.get_logger().warn(
                    f'{motion_name} 抓取点 3D 姿态被控制器拒绝，保持 xyz 不变，'
                    '改用固定抓取姿态再试一次。'
                )
                self.send_pose_movl(fallback_pose, '抓取点姿态兜底', 'picking')
                return
            self.finish_motion(f'{motion_name} {label} 未成功，停止当前流程。')
            return

        self.on_motion_accepted(motion_name)

    def on_motion_accepted(self, motion_name: str) -> None:
        if self.motion_stage == 'to_pre_grasp':
            self.get_logger().info(f'{motion_name} 预抓取点指令已发送，等待当前位姿到达预抓取点。')
        elif self.motion_stage == 'picking':
            self.get_logger().info(f'{motion_name} 抓取指令已发送，等待当前位姿进入夹爪闭合阈值。')
            self.start_pick_close_timeout()
        elif self.motion_stage == 'to_intermediate':
            self.get_logger().info(f'{motion_name} 中间点指令已发送，等待当前位姿到达中间点。')
        elif self.motion_stage == 'to_place':
            self.get_logger().info(f'{motion_name} 箱子点指令已发送，等待当前位姿到达箱子点。')
        elif self.motion_stage == 'to_camera':
            self.get_logger().info(f'{motion_name} 拍照点指令已发送，等待当前位姿回到拍照点。')

    def publish_gripper_stop(self) -> None:
        if self.gripper_stop_timer is not None:
            self.gripper_stop_timer.cancel()
            self.gripper_stop_timer = None

        command = Motor()
        command.id = int(self.get_parameter('gripper_id').value)
        command.dir = int(self.get_parameter('gripper_close_dir').value)
        command.mode = int(self.get_parameter('gripper_close_mode').value)
        command.speed = 0
        command.angle = 0
        command.state = 0
        command.sub_divide = int(self.get_parameter('gripper_sub_divide').value)
        self.gripper_pub.publish(command)
        self.get_logger().info('已发布夹爪停止命令。')

    def _call_simple_service(self, client, request, name: str, done_cb: Callable[[bool], None]) -> None:
        if not self._wait_for_service(client, name):
            done_cb(False)
            return

        future = client.call_async(request)

        def on_done(service_future) -> None:
            ok = self._service_ok(name, service_future)
            done_cb(ok)

        future.add_done_callback(on_done)

    def _service_ok(self, name: str, future) -> bool:
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(f'{name} 调用异常: {exc}')
            return False

        if response is None:
            self.get_logger().error(f'{name} 没有返回响应。')
            return False

        self.get_logger().info(f'{name} 返回: res={response.res}')
        return response.res == 0

    def _wait_for_service(self, client, service_name: str) -> bool:
        if client.service_is_ready():
            return True
        self.get_logger().info(f'等待服务: {service_name}')
        if client.wait_for_service(timeout_sec=2.0):
            return True
        self.get_logger().error(f'服务不可用: {service_name}')
        return False

    def finish_motion(self, reason: str) -> None:
        self.cancel_connection_wait()
        self.busy = False
        self.pending_pick_pose = None
        self.active_target_pose = None
        self.set_motion_stage('error')
        self.cancel_pick_close_timeout()
        if self.after_grasp_timer is not None:
            self.after_grasp_timer.cancel()
            self.after_grasp_timer = None
        self.get_logger().error(reason)

    def finish_sequence(self) -> None:
        self.cancel_connection_wait()
        self.busy = False
        self.pending_pick_pose = None
        self.active_target_pose = None
        self.cancel_pick_close_timeout()
        self.set_motion_stage('idle')
        self.get_logger().info('抓取流程完成。')

    def set_motion_stage(self, stage: str) -> None:
        self.motion_stage = stage
        self.publish_sequence_state()

    def publish_sequence_state(self) -> None:
        msg = String()
        msg.data = self.motion_stage
        self.sequence_state_pub.publish(msg)

    @staticmethod
    def _parse_pose_parameter(value: str, name: str) -> Optional[PoseStamped]:
        if not value.strip():
            return None

        parts = [part.strip() for part in value.replace(';', ',').split(',') if part.strip()]
        if len(parts) != 7:
            raise ValueError(f'{name} 必须是 7 个数字: x,y,z,qx,qy,qz,qw')

        numbers = [float(part) for part in parts]
        msg = PoseStamped()
        msg.header.frame_id = 'dobot_base'
        msg.pose.position.x = numbers[0]
        msg.pose.position.y = numbers[1]
        msg.pose.position.z = numbers[2]
        msg.pose.orientation.x = numbers[3]
        msg.pose.orientation.y = numbers[4]
        msg.pose.orientation.z = numbers[5]
        msg.pose.orientation.w = numbers[6]
        return msg

    @staticmethod
    def _parse_vector_parameter(value: str, name: str) -> tuple[float, float, float]:
        parts = [part.strip() for part in value.replace(';', ',').split(',') if part.strip()]
        if len(parts) != 3:
            raise ValueError(f'{name} 必须是 3 个数字: x,y,z')

        vector = tuple(float(part) for part in parts)
        norm = math.sqrt(sum(component * component for component in vector))
        if norm < 1e-9:
            raise ValueError(f'{name} 不能是零向量')
        return tuple(component / norm for component in vector)

    @staticmethod
    def _parse_vector4_parameter(value: str, name: str) -> tuple[float, float, float, float]:
        parts = [part.strip() for part in value.replace(';', ',').split(',') if part.strip()]
        if len(parts) != 4:
            raise ValueError(f'{name} 必须是 4 个数字: x,y,z,w')

        vector = tuple(float(part) for part in parts)
        norm = math.sqrt(sum(component * component for component in vector))
        if norm < 1e-9:
            raise ValueError(f'{name} 不能是零四元数')
        return tuple(component / norm for component in vector)

    @staticmethod
    def _parse_bool_parameter(value) -> bool:
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in ('1', 'true', 'yes', 'on')
        return bool(value)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TargetPoseGripper()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

import json
import logging
import socket
import time
from collections import deque

import numpy as np
from scipy.spatial.transform import Rotation


def _parse_float_list(value, expected_len, name):
  parts = [part.strip() for part in str(value).replace(';', ',').split(',') if part.strip()]
  if len(parts) != expected_len:
    raise ValueError(f'{name} must contain {expected_len} numbers, got {len(parts)}: {value}')
  return np.asarray([float(part) for part in parts], dtype=np.float64)


def _load_transform_from_args(args):
  if args.dobot_eye_in_hand:
    return None

  if args.dobot_base_T_cam_file:
    matrix = np.loadtxt(args.dobot_base_T_cam_file, dtype=np.float64)
    matrix = np.asarray(matrix, dtype=np.float64).reshape(4, 4)
    logging.info(f'loaded Dobot base_T_cam from {args.dobot_base_T_cam_file}:\n{matrix}')
    return matrix

  if args.dobot_base_T_cam:
    matrix = _parse_float_list(args.dobot_base_T_cam, 16, 'dobot_base_T_cam').reshape(4, 4)
    logging.info(f'loaded Dobot base_T_cam from command line:\n{matrix}')
    return matrix

  if args.dobot_allow_identity_handeye:
    logging.warning('using identity base_T_cam because --dobot_allow_identity_handeye is set')
    return np.eye(4, dtype=np.float64)

  raise RuntimeError(
    'Dobot fixed-camera target publishing needs --dobot_base_T_cam_file or --dobot_base_T_cam. '
    'Eye-in-hand mode uses --dobot_eye_in_hand 1 and does not need base_T_cam.'
  )


def _transform_from_rt(rotation_value, translation_value):
  matrix = np.eye(4, dtype=np.float64)
  matrix[:3, :3] = _parse_float_list(rotation_value, 9, 'dobot_R_cam2gripper').reshape(3, 3)
  matrix[:3, 3] = _parse_float_list(translation_value, 3, 'dobot_t_cam2gripper')
  return matrix


def _pose_dict_to_matrix(pose):
  matrix = np.eye(4, dtype=np.float64)
  matrix[:3, 3] = [float(pose['x']), float(pose['y']), float(pose['z'])]
  quat_xyzw = [float(pose['qx']), float(pose['qy']), float(pose['qz']), float(pose['qw'])]
  matrix[:3, :3] = Rotation.from_quat(quat_xyzw).as_matrix()
  return matrix


def _normalized(vector, fallback):
  vector = np.asarray(vector, dtype=np.float64)
  norm = np.linalg.norm(vector)
  if norm < 1e-9:
    vector = np.asarray(fallback, dtype=np.float64)
    norm = np.linalg.norm(vector)
  return vector / max(norm, 1e-9)


def _project_to_plane(vector, normal):
  vector = np.asarray(vector, dtype=np.float64)
  normal = _normalized(normal, [0, 0, 1])
  return vector - normal * np.dot(vector, normal)


def _signed_angle_about_axis(src, dst, axis):
  src = _normalized(src, [1, 0, 0])
  dst = _normalized(dst, [1, 0, 0])
  axis = _normalized(axis, [0, 0, 1])
  sin_value = np.dot(axis, np.cross(src, dst))
  cos_value = np.clip(np.dot(src, dst), -1.0, 1.0)
  return float(np.arctan2(sin_value, cos_value))


def _basis_from_reference_and_approach(reference_axis, approach_axis):
  reference_axis = _normalized(reference_axis, [1, 0, 0])
  approach_axis = _project_to_plane(approach_axis, reference_axis)
  if np.linalg.norm(approach_axis) < 1e-6:
    approach_axis = _project_to_plane([0, 0, 1], reference_axis)
  if np.linalg.norm(approach_axis) < 1e-6:
    approach_axis = _project_to_plane([0, 1, 0], reference_axis)
  approach_axis = _normalized(approach_axis, [0, 0, 1])
  side_axis = _normalized(np.cross(approach_axis, reference_axis), [0, 1, 0])
  approach_axis = _normalized(np.cross(reference_axis, side_axis), [0, 0, 1])
  return np.column_stack([reference_axis, side_axis, approach_axis])


class DobotTargetBridge:
  """UDP bridge for conda FoundationPose -> ROS 2 Dobot.

  ROS 2 Jazzy's rclpy is built for system Python 3.12, while FoundationPose runs
  in a Python 3.10 conda environment. This class intentionally uses only UDP and
  JSON; a ROS-side node in dobot_pick_place handles rclpy.
  """

  def __init__(self, args):
    self.base_T_cam = _load_transform_from_args(args)
    self.eye_in_hand = bool(args.dobot_eye_in_hand)
    self.gripper_T_cam = _transform_from_rt(args.dobot_R_cam2gripper,
                                            args.dobot_t_cam2gripper)
    self.base_T_gripper = None
    self.robot_state = 'unknown'
    self.wait_for_idle = bool(args.dobot_wait_for_idle)
    self.idle_states = set(part.strip() for part in args.dobot_idle_states.split(',') if part.strip())
    self.publish_cooldown = float(args.dobot_publish_cooldown)
    self.last_publish_time = -1e9
    self.stable_frames = max(1, int(args.dobot_pose_stable_frames))
    self.max_translation_jitter = float(args.dobot_pose_max_translation_jitter)
    self.translation_window = deque(maxlen=self.stable_frames)
    self.grasp_offset_obj = _parse_float_list(args.dobot_grasp_offset_obj, 3, 'dobot_grasp_offset_obj')
    self.tcp_to_tip = _parse_float_list(args.dobot_tcp_to_tip, 3, 'dobot_tcp_to_tip')
    self.use_object_orientation = bool(args.dobot_use_object_orientation)
    self.orientation_mode = str(args.dobot_grasp_orientation_mode).strip().lower()
    if self.use_object_orientation and self.orientation_mode == 'fixed':
      self.orientation_mode = 'object'
    self.fixed_quat_xyzw = _parse_float_list(args.dobot_grasp_quat_xyzw, 4, 'dobot_grasp_quat_xyzw')
    self.fixed_quat_xyzw = self.fixed_quat_xyzw / np.linalg.norm(self.fixed_quat_xyzw)
    self.object_axis_obj = _normalized(
      _parse_float_list(args.dobot_grasp_object_axis_obj, 3, 'dobot_grasp_object_axis_obj'),
      [0, 1, 0],
    )
    self.reference_axis_tcp = _normalized(
      _parse_float_list(args.dobot_grasp_reference_axis_tcp, 3, 'dobot_grasp_reference_axis_tcp'),
      [1, 0, 0],
    )
    self.min_target_distance = float(args.dobot_min_target_distance)
    self.max_target_distance = float(args.dobot_max_target_distance)
    self.target_frame_id = args.dobot_target_frame_id
    self.last_publish_reason = ''
    self.last_reason_log_time = 0.0

    self.target_addr = (args.dobot_udp_target_host, int(args.dobot_udp_target_port))
    self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    self.sock.setblocking(False)
    self.status_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    self.status_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    self.status_sock.bind((args.dobot_udp_status_bind_host, int(args.dobot_udp_status_port)))
    self.status_sock.setblocking(False)

    logging.info(
      f'Dobot UDP bridge enabled: target_udp={self.target_addr}, '
      f'status_udp={args.dobot_udp_status_bind_host}:{args.dobot_udp_status_port}, '
      f'frame={self.target_frame_id}, orientation_mode={self.orientation_mode}'
    )
    if self.eye_in_hand:
      logging.info(f'eye-in-hand enabled: gripper_T_cam=\n{self.gripper_T_cam}')

  def _poll_status(self):
    while True:
      try:
        data, _addr = self.status_sock.recvfrom(65535)
      except BlockingIOError:
        return
      try:
        msg = json.loads(data.decode('utf-8'))
      except json.JSONDecodeError:
        continue

      state = msg.get('state')
      if state and state != self.robot_state:
        logging.info(f'Dobot sequence state: {state}')
      if state:
        self.robot_state = str(state)
      if msg.get('current_pose') is not None:
        self.base_T_gripper = _pose_dict_to_matrix(msg['current_pose'])

  def _is_robot_idle(self):
    return (not self.wait_for_idle) or self.robot_state in self.idle_states

  def _is_pose_stable(self, target_xyz):
    self.translation_window.append(np.asarray(target_xyz, dtype=np.float64))
    if len(self.translation_window) < self.stable_frames:
      self.last_publish_reason = f'waiting stable pose {len(self.translation_window)}/{self.stable_frames}'
      return False

    points = np.asarray(self.translation_window)
    jitter = np.linalg.norm(points - points.mean(axis=0), axis=1).max()
    if jitter > self.max_translation_jitter:
      self.last_publish_reason = f'pose jitter {jitter:.4f} m > {self.max_translation_jitter:.4f} m'
      return False
    return True

  def _planar_grasp_quat(self, ob_in_base):
    fixed_rotation = Rotation.from_quat(self.fixed_quat_xyzw).as_matrix()
    approach_axis_base = fixed_rotation @ _normalized(self.tcp_to_tip, [0, 0, 1])
    object_axis_base = ob_in_base[:3, :3] @ self.object_axis_obj
    reference_axis_base = fixed_rotation @ self.reference_axis_tcp

    object_projected = _project_to_plane(object_axis_base, approach_axis_base)
    reference_projected = _project_to_plane(reference_axis_base, approach_axis_base)
    if np.linalg.norm(object_projected) < 1e-6 or np.linalg.norm(reference_projected) < 1e-6:
      logging.warning('planar grasp orientation fallback to fixed quaternion because axis projection is degenerate')
      return self.fixed_quat_xyzw, 0.0

    # A bottle has a bidirectional long axis for grasping, so avoid unnecessary
    # 180 degree wrist flips by aligning to the nearer of +axis and -axis.
    if np.dot(_normalized(reference_projected, [1, 0, 0]),
              _normalized(object_projected, [1, 0, 0])) < 0:
      object_projected = -object_projected

    angle = _signed_angle_about_axis(reference_projected, object_projected, approach_axis_base)
    delta_rotation = Rotation.from_rotvec(angle * _normalized(approach_axis_base, [0, 0, 1])).as_matrix()
    grasp_rotation = delta_rotation @ fixed_rotation
    return Rotation.from_matrix(grasp_rotation).as_quat(), float(np.degrees(angle))

  def _grasp_3d_quat(self, ob_in_base):
    fixed_rotation = Rotation.from_quat(self.fixed_quat_xyzw).as_matrix()
    fixed_approach_base = fixed_rotation @ _normalized(self.tcp_to_tip, [0, 0, 1])
    fixed_reference_base = fixed_rotation @ self.reference_axis_tcp
    object_axis_base = _normalized(ob_in_base[:3, :3] @ self.object_axis_obj, [0, 1, 0])

    # The bottle long axis is bidirectional for grasping. Pick the sign that
    # keeps the wrist closest to the known-good fixed grasp orientation.
    if np.dot(object_axis_base, fixed_reference_base) < 0:
      object_axis_base = -object_axis_base

    approach_axis_base = _project_to_plane(fixed_approach_base, object_axis_base)
    if np.linalg.norm(approach_axis_base) < 1e-6:
      approach_axis_base = _project_to_plane(ob_in_base[:3, :3] @ [0, 0, 1], object_axis_base)
    approach_axis_base = _normalized(approach_axis_base, fixed_approach_base)

    tcp_basis = _basis_from_reference_and_approach(
      self.reference_axis_tcp,
      _normalized(self.tcp_to_tip, [0, 0, 1]),
    )
    base_basis = _basis_from_reference_and_approach(object_axis_base, approach_axis_base)
    grasp_rotation = base_basis @ tcp_basis.T
    tilt_angle = np.degrees(
      np.arccos(np.clip(np.dot(_normalized(fixed_approach_base, [0, 0, 1]), approach_axis_base), -1.0, 1.0))
    )
    return Rotation.from_matrix(grasp_rotation).as_quat(), float(tilt_angle)

  def _make_target_pose(self, ob_in_cam):
    ob_in_cam = np.asarray(ob_in_cam, dtype=np.float64).reshape(4, 4)
    if self.eye_in_hand:
      if self.base_T_gripper is None:
        self.last_publish_reason = 'waiting current gripper pose from UDP status bridge'
        return None, None, None
      base_T_cam = self.base_T_gripper @ self.gripper_T_cam
    else:
      base_T_cam = self.base_T_cam

    ob_in_base = base_T_cam @ ob_in_cam
    grasp_xyz = ob_in_base[:3, 3] + ob_in_base[:3, :3] @ self.grasp_offset_obj

    planar_angle_deg = None
    if self.orientation_mode == 'object':
      quat_xyzw = Rotation.from_matrix(ob_in_base[:3, :3]).as_quat()
    elif self.orientation_mode in ('3d', 'three_d', 'spatial'):
      quat_xyzw, planar_angle_deg = self._grasp_3d_quat(ob_in_base)
    elif self.orientation_mode == 'planar':
      quat_xyzw, planar_angle_deg = self._planar_grasp_quat(ob_in_base)
    else:
      quat_xyzw = self.fixed_quat_xyzw

    tcp_rotation = Rotation.from_quat(quat_xyzw).as_matrix()
    target_xyz = grasp_xyz - tcp_rotation @ self.tcp_to_tip
    return target_xyz, quat_xyzw, planar_angle_deg

  def publish_if_ready(self, ob_in_cam):
    self._poll_status()
    now = time.monotonic()
    if not self._is_robot_idle():
      self.translation_window.clear()
      self.last_publish_reason = f'robot state is {self.robot_state}, wait for idle'
      self._log_wait_reason(now)
      return False

    if now - self.last_publish_time < self.publish_cooldown:
      self.last_publish_reason = 'publish cooldown'
      self._log_wait_reason(now)
      return False

    target_xyz, quat_xyzw, planar_angle_deg = self._make_target_pose(ob_in_cam)
    if target_xyz is None:
      self._log_wait_reason(now)
      return False

    target_distance = float(np.linalg.norm(target_xyz))
    if target_distance < self.min_target_distance or target_distance > self.max_target_distance:
      self.last_publish_reason = (
        f'target distance {target_distance:.3f} m outside '
        f'[{self.min_target_distance:.3f}, {self.max_target_distance:.3f}]'
      )
      self._log_wait_reason(now)
      return False

    if not self._is_pose_stable(target_xyz):
      self._log_wait_reason(now)
      return False

    msg = {
      'frame_id': self.target_frame_id,
      'x': float(target_xyz[0]),
      'y': float(target_xyz[1]),
      'z': float(target_xyz[2]),
      'qx': float(quat_xyzw[0]),
      'qy': float(quat_xyzw[1]),
      'qz': float(quat_xyzw[2]),
      'qw': float(quat_xyzw[3]),
      'stamp': time.time(),
    }
    payload = json.dumps(msg, separators=(',', ':')).encode('utf-8')
    self.sock.sendto(payload, self.target_addr)
    self.last_publish_time = now
    self.translation_window.clear()
    self.last_publish_reason = 'published'
    logging.info(
      'published Dobot UDP target: '
      f'x={target_xyz[0]:.4f}, y={target_xyz[1]:.4f}, z={target_xyz[2]:.4f}, '
      f'q=({quat_xyzw[0]:.4f},{quat_xyzw[1]:.4f},{quat_xyzw[2]:.4f},{quat_xyzw[3]:.4f}), '
      f'tcp_to_tip=({self.tcp_to_tip[0]:.3f},{self.tcp_to_tip[1]:.3f},{self.tcp_to_tip[2]:.3f}), '
      f'orientation_mode={self.orientation_mode}, '
      f'planar_angle_deg={planar_angle_deg if planar_angle_deg is not None else 0.0:.1f}'
    )
    return True

  def _log_wait_reason(self, now):
    if now - self.last_reason_log_time < 1.0:
      return
    self.last_reason_log_time = now
    logging.info(
      'Dobot target not published yet: '
      f'{self.last_publish_reason}; state={self.robot_state}; '
      f'has_current_pose={self.base_T_gripper is not None}'
    )

  def close(self):
    self.sock.close()
    self.status_sock.close()


def build_dobot_target_bridge(args):
  if not args.dobot_publish_target:
    return None
  return DobotTargetBridge(args)

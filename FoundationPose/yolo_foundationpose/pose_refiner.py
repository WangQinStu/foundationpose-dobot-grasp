import os,logging,importlib.util,sys,csv
import inspect
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.spatial import cKDTree

from yolo_foundationpose.geometry import pose_matches_detection


def _rotation_delta_deg(src_pose, dst_pose):
  delta_R = dst_pose[:3,:3]@src_pose[:3,:3].T
  trace = np.clip((np.trace(delta_R)-1.0)/2.0, -1.0, 1.0)
  return float(np.degrees(np.arccos(trace)))


class SdfrPoseRefiner:
  def __init__(self, args, est):
    self.args = args
    self.est = est
    self.enabled = bool(args.use_sdfr)
    self.backend = 'disabled'
    self.refine_fn = None
    self.options_builder = None
    self.sdfr_root = None

    if not self.enabled:
      return

    self.sdfr_root = self._resolve_sdfr_root(args.sdfr_root)
    self.refine_fn = self._load_external_sdfr(self.sdfr_root)
    if self.refine_fn is None:
      self.backend = 'unavailable'
      logging.warning('SDFR requested but no callable refine_pose_with_sdfr was found; pose refinement will be skipped')
    else:
      self.options_builder = self._load_options_builder(self.sdfr_root)
      self.backend = 'external_sdfr'
      logging.info(f'SDFR backend ready: {self.backend}, root={self.sdfr_root}')


  def _resolve_sdfr_root(self, configured_root):
    candidates = []
    if configured_root:
      candidates.append(os.path.abspath(configured_root))

    code_dir = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
    candidates.extend([
      os.path.abspath(os.path.join(code_dir, '..', 'SDFR')),
      os.path.abspath(os.path.join(code_dir, '..', '..', 'SDFR')),
    ])

    env_root = os.environ.get('SDFR_ROOT')
    if env_root:
      candidates.append(os.path.abspath(env_root))

    for root in candidates:
      if os.path.isfile(os.path.join(root, 'my_tools', 'inference.py')):
        return root

    return candidates[0] if len(candidates)>0 else None


  def _load_external_sdfr(self, sdfr_root):
    if sdfr_root is None:
      return None

    inference_path = os.path.join(sdfr_root, 'my_tools', 'inference.py')
    if not os.path.isfile(inference_path):
      logging.warning(f'SDFR inference entry not found: {inference_path}')
      return None

    try:
      spec = importlib.util.spec_from_file_location('foundationpose_sdfr_inference', inference_path)
      if spec is None or spec.loader is None:
        return None
      module = importlib.util.module_from_spec(spec)
      sys.modules[spec.name] = module
      spec.loader.exec_module(module)
      self._module = module
      return getattr(module, 'refine_pose_with_sdfr', None)
    except Exception as e:
      logging.warning(f'Failed to load SDFR inference module: {e}')
      return None


  def _load_options_builder(self, sdfr_root):
    module = getattr(self, '_module', None)
    if module is None:
      return None
    return getattr(module, 'build_sdfr_options_from_args', None)


  def empty_info(self):
    return {
      'enabled': self.enabled,
      'backend': self.backend,
      'applied': False,
      'status': 'disabled' if not self.enabled else 'idle',
      'rot_delta_deg': 0.0,
      'trans_delta_mm': 0.0,
      'score_before_mm': None,
      'score_after_mm': None,
      'input_pose': None,
      'output_pose': None,
      'raw_output_pose': None,
      'raw_rot_delta_deg': 0.0,
      'raw_trans_delta_mm': 0.0,
      'pose_compare': None,
      'test_perturb_applied': False,
    }


  def _pose_points_score(self, pose_full, observed_pts):
    if observed_pts is None or len(observed_pts) == 0:
      return None
    centered_pose = self.est.full_pose_to_centered(pose_full)
    model_pts = self.est.pts.detach().cpu().numpy()
    pred_pts = (centered_pose[:3,:3] @ model_pts.T).T + centered_pose[:3,3]
    tree = cKDTree(pred_pts)
    dists, _ = tree.query(observed_pts, k=1)
    return float(np.mean(dists))


  def _backproject_masked_points(self, depth, mask, K, max_points):
    valid = (depth > 1e-6) & (mask > 0)
    ys, xs = np.nonzero(valid)
    if len(xs) == 0:
      return np.zeros((0,3), dtype=np.float32)
    z = depth[ys, xs].astype(np.float32)
    fx, fy = float(K[0,0]), float(K[1,1])
    cx, cy = float(K[0,2]), float(K[1,2])
    x = (xs.astype(np.float32) - cx) * z / fx
    y = (ys.astype(np.float32) - cy) * z / fy
    pts = np.stack([x, y, z], axis=1).astype(np.float32)
    if len(pts) > max_points:
      ids = np.random.choice(len(pts), size=max_points, replace=False)
      pts = pts[ids]
    return pts


  def refine(self, rgb, depth, mask, pose, K, mesh_file, frame_id=None,
             is_register=False, best=None, to_origin=None, bbox=None):
    info = self.empty_info()
    if pose is None:
      info['status'] = 'no_pose'
      return pose, info
    original_pose = np.asarray(pose, dtype=np.float32).reshape(4,4).copy()
    pose_for_sdfr = self._apply_test_perturbation(original_pose)
    info['test_perturb_applied'] = not np.allclose(original_pose, pose_for_sdfr)
    info['input_pose'] = pose_for_sdfr.copy()
    info['output_pose'] = info['input_pose'].copy()
    if not self.enabled:
      return pose, info
    if self.refine_fn is None:
      info['status'] = 'unavailable'
      return pose, info
    if mask is None or int(mask.sum())<=0:
      info['status'] = 'mask_empty'
      return pose, info
    if bool(getattr(self.args, 'sdfr_register_only', 1)) and not is_register:
      info['status'] = 'skipped_track_frame'
      return pose, info
    interval = max(1, int(getattr(self.args, 'sdfr_refine_interval', 1)))
    if frame_id is not None and frame_id % interval != 0:
      info['status'] = f'skipped_interval_{interval}'
      return pose, info

    try:
      observed_pts = self._backproject_masked_points(
        depth, mask, K, max_points=int(getattr(self.args, 'sdfr_max_points', 2500))
      )
      if len(observed_pts) < 64:
        info['status'] = 'too_few_points'
        return pose, info

      score_before = self._pose_points_score(pose_for_sdfr, observed_pts)
      if score_before is not None:
        info['score_before_mm'] = score_before * 1000.0

      kwargs = {}
      if self.options_builder is not None:
        try:
          kwargs['options'] = self.options_builder(self.args)
        except Exception as e:
          logging.warning(f'Failed to build SDFR options from args: {e}')
      elif self._accepts_options_kw():
        kwargs['options'] = None

      refined_pose = self.refine_fn(rgb, depth, mask, pose_for_sdfr.copy(), K, mesh_file, **kwargs)
      refined_pose = np.asarray(refined_pose, dtype=np.float32).reshape(4,4)
      if not np.isfinite(refined_pose).all():
        raise ValueError('refined pose contains non-finite values')
      refined_pose[3] = np.asarray([0,0,0,1], dtype=np.float32)

      info['applied'] = True
      info['status'] = 'refined'
      info['raw_output_pose'] = refined_pose.copy()
      info['output_pose'] = refined_pose.copy()
      info['rot_delta_deg'] = _rotation_delta_deg(pose_for_sdfr, refined_pose)
      info['trans_delta_mm'] = float(np.linalg.norm(refined_pose[:3,3]-pose_for_sdfr[:3,3])*1000.0)
      info['raw_rot_delta_deg'] = info['rot_delta_deg']
      info['raw_trans_delta_mm'] = info['trans_delta_mm']
      score_after = self._pose_points_score(refined_pose, observed_pts)
      if score_after is not None:
        info['score_after_mm'] = score_after * 1000.0
      info['pose_compare'] = self._build_pose_compare(info)

      if info['trans_delta_mm'] > float(getattr(self.args, 'sdfr_accept_max_translation_mm', 20.0)):
        info['status'] = 'rejected_translation_jump'
        info['applied'] = False
        info['output_pose'] = self._display_pose_for_rejection(info)
        if info['test_perturb_applied'] and not bool(getattr(self.args, 'sdfr_show_rejected_pose', 0)):
          info['output_pose'] = original_pose.copy()
        info['pose_compare'] = self._build_pose_compare(info)
        self._log_rejected(info)
        self._write_pose_compare(frame_id, info)
        return original_pose, info

      if info['rot_delta_deg'] > float(getattr(self.args, 'sdfr_accept_max_rotation_deg', 8.0)):
        info['status'] = 'rejected_rotation_jump'
        info['applied'] = False
        info['output_pose'] = self._display_pose_for_rejection(info)
        if info['test_perturb_applied'] and not bool(getattr(self.args, 'sdfr_show_rejected_pose', 0)):
          info['output_pose'] = original_pose.copy()
        info['pose_compare'] = self._build_pose_compare(info)
        self._log_rejected(info)
        self._write_pose_compare(frame_id, info)
        return original_pose, info

      if best is not None and bbox is not None and to_origin is not None:
        if not pose_matches_detection(refined_pose, K, best['candidate'],
                                      to_origin=to_origin,
                                      bbox=bbox,
                                      slack=self.args.pose_bbox_slack,
                                      depth_tolerance=self.args.pose_depth_tolerance,
                                      min_projected_iou=self.args.pose_min_projected_iou):
          info['status'] = 'rejected_detection_mismatch'
          info['applied'] = False
          info['output_pose'] = self._display_pose_for_rejection(info)
          if info['test_perturb_applied'] and not bool(getattr(self.args, 'sdfr_show_rejected_pose', 0)):
            info['output_pose'] = original_pose.copy()
          info['pose_compare'] = self._build_pose_compare(info)
          self._log_rejected(info)
          self._write_pose_compare(frame_id, info)
          return original_pose, info

      if score_before is not None and score_after is not None:
        ratio = float(getattr(self.args, 'sdfr_accept_score_ratio', 0.98))
        if score_after > score_before * ratio:
          info['status'] = 'rejected_no_alignment_gain'
          info['applied'] = False
          info['output_pose'] = self._display_pose_for_rejection(info)
          if info['test_perturb_applied'] and not bool(getattr(self.args, 'sdfr_show_rejected_pose', 0)):
            info['output_pose'] = original_pose.copy()
          info['pose_compare'] = self._build_pose_compare(info)
          self._log_rejected(info)
          self._write_pose_compare(frame_id, info)
          return original_pose, info

      # The tracker consumes est.pose_last in the centered-mesh frame.
      self.est.set_pose_last_from_full_pose(refined_pose)
      self._write_pose_compare(frame_id, info)
      logging.info(f'SDFR refined pose: dR={info["rot_delta_deg"]:.3f} deg, dT={info["trans_delta_mm"]:.2f} mm')
      return refined_pose, info
    except Exception as e:
      info['status'] = f'failed:{type(e).__name__}'
      logging.warning(f'SDFR refinement failed: {e}')
      info['output_pose'] = original_pose.copy() if info.get('test_perturb_applied') else info['input_pose'].copy()
      return original_pose, info


  def _parse_triplet(self, text, name):
    try:
      values = [float(v.strip()) for v in str(text).split(',')]
    except Exception as e:
      raise ValueError(f'invalid {name}, expected a,b,c') from e
    if len(values) != 3:
      raise ValueError(f'invalid {name}, expected exactly 3 comma-separated values')
    return np.asarray(values, dtype=np.float32)


  def _apply_test_perturbation(self, pose):
    if not bool(getattr(self.args, 'sdfr_test_perturb', 0)):
      return pose.copy()
    xyz_mm = self._parse_triplet(getattr(self.args, 'sdfr_test_perturb_xyz_mm', '0,0,0'),
                                 'sdfr_test_perturb_xyz_mm')
    rpy_deg = self._parse_triplet(getattr(self.args, 'sdfr_test_perturb_rpy_deg', '0,0,0'),
                                  'sdfr_test_perturb_rpy_deg')
    delta = np.eye(4, dtype=np.float32)
    delta[:3,:3] = Rotation.from_euler('xyz', rpy_deg, degrees=True).as_matrix().astype(np.float32)
    delta[:3,3] = xyz_mm / 1000.0
    return (pose @ delta).astype(np.float32)


  def _pose_values(self, pose):
    pose = np.asarray(pose, dtype=np.float64).reshape(4,4)
    quat = Rotation.from_matrix(pose[:3,:3]).as_quat()
    euler = Rotation.from_matrix(pose[:3,:3]).as_euler('xyz', degrees=True)
    return {
      'x_mm': float(pose[0,3] * 1000.0),
      'y_mm': float(pose[1,3] * 1000.0),
      'z_mm': float(pose[2,3] * 1000.0),
      'roll_deg': float(euler[0]),
      'pitch_deg': float(euler[1]),
      'yaw_deg': float(euler[2]),
      'qx': float(quat[0]),
      'qy': float(quat[1]),
      'qz': float(quat[2]),
      'qw': float(quat[3]),
    }


  def _build_pose_compare(self, info):
    before = info.get('input_pose')
    candidate = info.get('raw_output_pose')
    if candidate is None:
      candidate = info.get('output_pose')
    used = info.get('output_pose')
    if before is None or candidate is None or used is None:
      return None

    before_vals = self._pose_values(before)
    candidate_vals = self._pose_values(candidate)
    used_vals = self._pose_values(used)
    return {
      'before': before_vals,
      'candidate': candidate_vals,
      'used': used_vals,
      'candidate_delta': {
        'dx_mm': candidate_vals['x_mm'] - before_vals['x_mm'],
        'dy_mm': candidate_vals['y_mm'] - before_vals['y_mm'],
        'dz_mm': candidate_vals['z_mm'] - before_vals['z_mm'],
        'dT_mm': float(info.get('raw_trans_delta_mm', 0.0)),
        'dR_deg': float(info.get('raw_rot_delta_deg', 0.0)),
      },
      'used_delta': {
        'dx_mm': used_vals['x_mm'] - before_vals['x_mm'],
        'dy_mm': used_vals['y_mm'] - before_vals['y_mm'],
        'dz_mm': used_vals['z_mm'] - before_vals['z_mm'],
        'dT_mm': float(np.linalg.norm(np.asarray(used)[:3,3] - np.asarray(before)[:3,3]) * 1000.0),
        'dR_deg': _rotation_delta_deg(before, used),
      },
    }


  def _write_pose_compare(self, frame_id, info):
    if not bool(getattr(self.args, 'sdfr_save_pose_compare', 1)):
      return
    compare = info.get('pose_compare')
    if compare is None:
      return

    path = getattr(self.args, 'sdfr_pose_compare_file', None)
    if not path:
      path = os.path.join(self.args.debug_dir, 'sdfr_pose_compare.csv')
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)

    fieldnames = [
      'frame_id', 'status', 'applied', 'test_perturb_applied',
      'before_x_mm', 'before_y_mm', 'before_z_mm',
      'candidate_x_mm', 'candidate_y_mm', 'candidate_z_mm',
      'used_x_mm', 'used_y_mm', 'used_z_mm',
      'candidate_dx_mm', 'candidate_dy_mm', 'candidate_dz_mm',
      'candidate_dT_mm', 'candidate_dR_deg',
      'used_dx_mm', 'used_dy_mm', 'used_dz_mm', 'used_dT_mm', 'used_dR_deg',
      'before_roll_deg', 'before_pitch_deg', 'before_yaw_deg',
      'candidate_roll_deg', 'candidate_pitch_deg', 'candidate_yaw_deg',
      'used_roll_deg', 'used_pitch_deg', 'used_yaw_deg',
      'before_qx', 'before_qy', 'before_qz', 'before_qw',
      'candidate_qx', 'candidate_qy', 'candidate_qz', 'candidate_qw',
      'used_qx', 'used_qy', 'used_qz', 'used_qw',
      'score_before_mm', 'score_after_mm',
    ]
    row = {
      'frame_id': -1 if frame_id is None else int(frame_id),
      'status': info.get('status'),
      'applied': bool(info.get('applied', False)),
      'test_perturb_applied': bool(info.get('test_perturb_applied', False)),
      'score_before_mm': info.get('score_before_mm'),
      'score_after_mm': info.get('score_after_mm'),
    }
    for name, values in [('before', compare['before']), ('candidate', compare['candidate']), ('used', compare['used'])]:
      for k, v in values.items():
        row[f'{name}_{k}'] = v
    for name, values in [('candidate', compare['candidate_delta']), ('used', compare['used_delta'])]:
      for k, v in values.items():
        row[f'{name}_{k}'] = v

    write_header = not os.path.exists(path)
    with open(path, 'a', newline='') as f:
      writer = csv.DictWriter(f, fieldnames=fieldnames)
      if write_header:
        writer.writeheader()
      writer.writerow({k: row.get(k) for k in fieldnames})


  def _display_pose_for_rejection(self, info):
    if bool(getattr(self.args, 'sdfr_show_rejected_pose', 0)) and info.get('raw_output_pose') is not None:
      return info['raw_output_pose'].copy()
    return info['input_pose'].copy()


  def _log_rejected(self, info):
    logging.info(
      'SDFR candidate rejected: status=%s, raw_dR=%.3f deg, raw_dT=%.2f mm, '
      'score_before=%s mm, score_after=%s mm',
      info.get('status'),
      float(info.get('raw_rot_delta_deg', 0.0)),
      float(info.get('raw_trans_delta_mm', 0.0)),
      'None' if info.get('score_before_mm') is None else f'{info["score_before_mm"]:.2f}',
      'None' if info.get('score_after_mm') is None else f'{info["score_after_mm"]:.2f}',
    )


  def _accepts_options_kw(self):
    if self.refine_fn is None:
      return False
    try:
      sig = inspect.signature(self.refine_fn)
      if 'options' in sig.parameters:
        return True
      return any(p.kind == inspect.Parameter.VAR_KEYWORD for p in sig.parameters.values())
    except Exception:
      return False


def build_pose_refiner(args, est):
  return SdfrPoseRefiner(args, est)

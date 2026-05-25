import logging,inspect
from dataclasses import dataclass
from functools import lru_cache

import cv2
import numpy as np
import open3d as o3d
import trimesh
from scipy.optimize import least_squares

try:
  from pysdf import SDF as PySDF
except Exception:
  PySDF = None


@dataclass
class SdfrOptions:
  max_points: int = 2500
  max_iterations: int = 25
  sdf_trunc: float = 0.02
  depth_band: float = 0.12
  regularization: float = 0.002
  robust_scale: float = 0.01
  translation_limit: float = 0.05
  rotation_limit_deg: float = 12.0
  icp_enable: bool = True
  icp_max_iterations: int = 15
  icp_distance_threshold: float = 0.01
  voxel_size: float = 0.003


def _dict_to_options(options):
  if options is None:
    return SdfrOptions()
  if isinstance(options, SdfrOptions):
    return options
  return SdfrOptions(**options)


def _hat(rotvec):
  return cv2.Rodrigues(np.asarray(rotvec, dtype=np.float64).reshape(3))[0]


def _transform_points(pose, pts):
  return pts@pose[:3,:3].T + pose[:3,3]


def _inverse_transform_points(pose, pts):
  R = pose[:3,:3]
  t = pose[:3,3]
  return (pts-t.reshape(1,3))@R


def _depth_to_points(depth, mask, K):
  ys, xs = np.nonzero(mask.astype(np.uint8))
  if len(xs)==0:
    return np.zeros((0,3), dtype=np.float32)
  z = depth[ys, xs].astype(np.float32)
  valid = z > 1e-4
  if not np.any(valid):
    return np.zeros((0,3), dtype=np.float32)
  xs = xs[valid].astype(np.float32)
  ys = ys[valid].astype(np.float32)
  z = z[valid]
  fx, fy = float(K[0,0]), float(K[1,1])
  cx, cy = float(K[0,2]), float(K[1,2])
  x = (xs-cx)*z/fx
  y = (ys-cy)*z/fy
  return np.stack([x, y, z], axis=1).astype(np.float32)


def _crop_points(points, init_pose, opts):
  if len(points)==0:
    return points
  depth_center = float(init_pose[2,3])
  keep = np.abs(points[:,2]-depth_center) <= opts.depth_band
  points = points[keep]
  if len(points)==0:
    return points
  if len(points) > opts.max_points:
    ids = np.linspace(0, len(points)-1, opts.max_points).astype(np.int32)
    points = points[ids]
  return points


def _voxel_downsample(points, voxel_size):
  if len(points)==0:
    return points
  pcd = o3d.geometry.PointCloud()
  pcd.points = o3d.utility.Vector3dVector(points.astype(np.float64))
  pcd = pcd.voxel_down_sample(max(voxel_size, 1e-4))
  return np.asarray(pcd.points, dtype=np.float32)


@lru_cache(maxsize=8)
def _load_mesh_backend(mesh_file):
  mesh = trimesh.load(mesh_file, force='mesh')
  if not isinstance(mesh, trimesh.Trimesh):
    raise ValueError(f'expected Trimesh, got {type(mesh)} from {mesh_file}')
  mesh = mesh.copy()
  if len(mesh.vertices)==0 or len(mesh.faces)==0:
    raise ValueError(f'mesh is empty: {mesh_file}')
  query = trimesh.proximity.ProximityQuery(mesh)
  sdf_backend = None
  sdf_name = 'trimesh'
  if PySDF is not None:
    try:
      sdf_backend = PySDF(mesh.vertices, mesh.faces)
      sdf_name = 'pysdf'
    except Exception as e:
      logging.warning(f'PySDF backend unavailable for {mesh_file}: {e}')
  mesh_pcd = o3d.geometry.PointCloud()
  mesh_pcd.points = o3d.utility.Vector3dVector(np.asarray(mesh.vertices, dtype=np.float64))
  mesh_pcd.normals = o3d.utility.Vector3dVector(np.asarray(mesh.vertex_normals, dtype=np.float64))
  return mesh, query, sdf_backend, sdf_name, mesh_pcd


def _signed_distance(query, sdf_backend, points_obj):
  if len(points_obj)==0:
    return np.zeros((0,), dtype=np.float64)
  if sdf_backend is not None:
    return np.asarray(sdf_backend(points_obj), dtype=np.float64)
  return np.asarray(query.signed_distance(points_obj), dtype=np.float64)


def _sdf_residual(delta, init_pose, scene_points, query, sdf_backend, opts):
  rot = _hat(delta[:3])
  trans = delta[3:].reshape(3)
  pose = np.eye(4, dtype=np.float64)
  pose[:3,:3] = rot@init_pose[:3,:3]
  pose[:3,3] = rot@init_pose[:3,3] + trans

  pts_obj = _inverse_transform_points(pose, scene_points)
  sdf = _signed_distance(query, sdf_backend, pts_obj)
  if len(sdf)==0:
    sdf = np.zeros((1,), dtype=np.float64)

  sdf = np.clip(sdf, -opts.sdf_trunc, opts.sdf_trunc)
  denom = max(opts.robust_scale, 1e-5)
  robust = np.tanh(sdf/denom)
  reg = np.concatenate([
    delta[:3] / np.deg2rad(max(opts.rotation_limit_deg, 1e-3)),
    delta[3:] / max(opts.translation_limit, 1e-4),
  ]) * opts.regularization
  return np.concatenate([robust, reg], axis=0)


def _run_sdf_optimization(init_pose, scene_points, query, sdf_backend, opts):
  x0 = np.zeros((6,), dtype=np.float64)
  result = least_squares(
    _sdf_residual, x0,
    args=(init_pose.astype(np.float64), scene_points.astype(np.float64), query, sdf_backend, opts),
    method='trf',
    max_nfev=max(10, int(opts.max_iterations)),
    ftol=1e-4,
    xtol=1e-4,
    gtol=1e-4,
    loss='soft_l1',
  )
  delta = result.x
  rot = _hat(delta[:3])
  refined = np.eye(4, dtype=np.float64)
  refined[:3,:3] = rot@init_pose[:3,:3]
  refined[:3,3] = rot@init_pose[:3,3] + delta[3:]
  return refined, result


def _run_icp(mesh, init_pose, scene_points, opts):
  if len(scene_points)==0:
    return init_pose

  source = mesh.sample(max(4000, min(12000, len(scene_points)*4)))
  source = _transform_points(init_pose, source.astype(np.float64))
  source = _voxel_downsample(source.astype(np.float32), opts.voxel_size)
  target = _voxel_downsample(scene_points.astype(np.float32), opts.voxel_size)
  if len(source) < 30 or len(target) < 30:
    return init_pose

  source_pcd = o3d.geometry.PointCloud()
  source_pcd.points = o3d.utility.Vector3dVector(source.astype(np.float64))
  target_pcd = o3d.geometry.PointCloud()
  target_pcd.points = o3d.utility.Vector3dVector(target.astype(np.float64))
  radius = max(opts.voxel_size*2.0, 1e-3)
  source_pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=30))
  target_pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=30))

  reg = o3d.pipelines.registration.registration_icp(
    source_pcd,
    target_pcd,
    max(opts.icp_distance_threshold, opts.voxel_size*3.0),
    np.eye(4, dtype=np.float64),
    o3d.pipelines.registration.TransformationEstimationPointToPlane(),
    o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=max(1, int(opts.icp_max_iterations))),
  )
  return reg.transformation@init_pose


def refine_pose_with_sdfr(rgb, depth, mask, initial_pose, K, mesh_file, options=None):
  """
  使用 SDF + ICP 的局部优化，对 FoundationPose 初值做在线精修。

  输入 pose 为物体到相机坐标系 4x4 变换；返回值保持同一坐标系定义。
  """
  opts = _dict_to_options(options)
  initial_pose = np.asarray(initial_pose, dtype=np.float64).reshape(4,4)
  K = np.asarray(K, dtype=np.float64).reshape(3,3)
  depth = np.asarray(depth, dtype=np.float32)
  mask = np.asarray(mask).astype(np.uint8)

  scene_points = _depth_to_points(depth, mask, K)
  scene_points = _crop_points(scene_points, initial_pose, opts)
  scene_points = _voxel_downsample(scene_points, opts.voxel_size)
  if len(scene_points) < 50:
    logging.info('SDFR skipped: not enough masked depth points')
    return initial_pose.astype(np.float32)

  mesh, query, sdf_backend, sdf_name, _ = _load_mesh_backend(mesh_file)
  refined_pose, result = _run_sdf_optimization(initial_pose, scene_points, query, sdf_backend, opts)

  if opts.icp_enable:
    refined_pose = _run_icp(mesh, refined_pose, scene_points, opts)

  refined_pose = np.asarray(refined_pose, dtype=np.float32)
  refined_pose[3] = np.asarray([0,0,0,1], dtype=np.float32)
  logging.info(
    f'SDFR refine done: backend={sdf_name}, points={len(scene_points)}, '
    f'cost={float(result.cost):.6f}, nfev={int(result.nfev)}'
  )
  return refined_pose


def build_sdfr_options_from_args(args):
  fields = inspect.signature(SdfrOptions).parameters
  data = {}
  for name in fields:
    arg_name = f'sdfr_{name}'
    if hasattr(args, arg_name):
      data[name] = getattr(args, arg_name)
  if 'icp_enable' in data:
    data['icp_enable'] = bool(data['icp_enable'])
  return SdfrOptions(**data)

from estimater import *


def build_estimator(mesh_file, debug_dir, debug):
  # FoundationPose 需要目标物体 mesh。这里同时计算：
  # - to_origin：把 oriented bounding box 坐标转回 mesh 原始坐标的变换；
  # - bbox：用于可视化和 pose 一致性检查的 3D 包围盒。
  mesh = trimesh.load(mesh_file)
  to_origin, extents = trimesh.bounds.oriented_bounds(mesh)
  bbox = np.stack([-extents/2, extents/2], axis=0).reshape(2,3)

  # scorer 用于从多组初始 pose 中选最可信的结果；
  # refiner 用于根据 RGB-D crop 迭代修正 pose；
  # nvdiffrast 的 CUDA context 用于按候选 pose 渲染 mesh。
  scorer = ScorePredictor()
  refiner = PoseRefinePredictor()
  glctx = dr.RasterizeCudaContext()
  est = FoundationPose(model_pts=mesh.vertices, model_normals=mesh.vertex_normals,
                       mesh=mesh, scorer=scorer, refiner=refiner, debug_dir=debug_dir,
                       debug=debug, glctx=glctx)
  return est, to_origin, bbox

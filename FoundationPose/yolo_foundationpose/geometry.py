import numpy as np


def bbox_iou(a, b):
  ax1,ay1,ax2,ay2 = a.astype(float)
  bx1,by1,bx2,by2 = b.astype(float)
  ix1,iy1 = max(ax1,bx1), max(ay1,by1)
  ix2,iy2 = min(ax2,bx2), min(ay2,by2)
  iw,ih = max(0.0, ix2-ix1), max(0.0, iy2-iy1)
  inter = iw*ih
  area_a = max(1.0, (ax2-ax1)*(ay2-ay1))
  area_b = max(1.0, (bx2-bx1)*(by2-by1))
  return inter / max(1.0, area_a+area_b-inter)


def bbox_center(bbox):
  x1,y1,x2,y2 = bbox.astype(float)
  return np.array([(x1+x2)*0.5, (y1+y2)*0.5], dtype=np.float32)


def project_pose_center(pose, K, to_origin=None):
  if pose is None:
    return None, None
  if to_origin is not None:
    pose = pose @ np.linalg.inv(to_origin)
  center = pose[:3,3].reshape(3,1)
  if center[2,0] <= 0:
    return None, center
  uv = (K @ center).reshape(3)
  uv = uv[:2] / max(1e-6, uv[2])
  return uv, center


def project_bbox(pose, K, bbox, to_origin=None):
  if pose is None:
    return None
  if to_origin is not None:
    pose = pose @ np.linalg.inv(to_origin)

  bmin,bmax = bbox
  corners = np.array([[x,y,z] for x in [bmin[0], bmax[0]]
                              for y in [bmin[1], bmax[1]]
                              for z in [bmin[2], bmax[2]]], dtype=np.float32)
  pts = (pose[:3,:3] @ corners.T + pose[:3,3:4]).T
  if np.any(pts[:,2] <= 1e-6):
    return None

  uvs = (K @ pts.T).T
  uvs = uvs[:,:2] / np.maximum(uvs[:,2:3], 1e-6)
  x1,y1 = uvs.min(axis=0)
  x2,y2 = uvs.max(axis=0)
  return np.array([x1,y1,x2,y2], dtype=np.float32)


def point_in_mask(mask, uv, radius=5):
  if mask is None or uv is None:
    return True
  x,y = np.round(uv).astype(int)
  H,W = mask.shape[:2]
  if x < 0 or x >= W or y < 0 or y >= H:
    return False
  x1,x2 = max(0, x-radius), min(W, x+radius+1)
  y1,y2 = max(0, y-radius), min(H, y+radius+1)
  return bool(mask[y1:y2, x1:x2].any())


def pose_center_in_bbox(pose, K, bbox, to_origin=None, slack=1.2):
  uv, _ = project_pose_center(pose, K, to_origin=to_origin)
  if uv is None:
    return False
  x1,y1,x2,y2 = bbox.astype(float)
  w,h = x2-x1, y2-y1
  x1 -= w*slack
  x2 += w*slack
  y1 -= h*slack
  y2 += h*slack
  return x1 <= uv[0] <= x2 and y1 <= uv[1] <= y2


def pose_matches_detection(pose, K, cand, to_origin=None, bbox=None, slack=1.2,
                           depth_tolerance=0.25, min_projected_iou=0.03):
  uv, center = project_pose_center(pose, K, to_origin=to_origin)
  if uv is None:
    return False

  x1,y1,x2,y2 = cand.bbox.astype(float)
  w,h = x2-x1, y2-y1
  x1 -= w*slack
  x2 += w*slack
  y1 -= h*slack
  y2 += h*slack
  if not (x1 <= uv[0] <= x2 and y1 <= uv[1] <= y2):
    return False
  if not point_in_mask(getattr(cand, 'mask', None), uv):
    return False

  if bbox is not None:
    projected_bbox = project_bbox(pose, K, bbox, to_origin=to_origin)
    if projected_bbox is None:
      return False
    if bbox_iou(projected_bbox, cand.bbox) < min_projected_iou:
      return False

  # 深度差过大时，通常说明跟踪已经漂移到了别的物体或背景。
  median_depth = cand.score_detail.get('median_depth')
  if median_depth is not None and abs(float(center[2,0])-float(median_depth)) > depth_tolerance:
    return False
  return True

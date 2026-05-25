import cv2
import numpy as np


def refine_mask_for_pose(mask, depth, zmin, zmax, open_kernel=3, close_kernel=5,
                         erode_kernel=3, depth_band=0.06, depth_mad_scale=3.0,
                         min_area=400, anchor_xy=None):
  mask = mask.astype(np.uint8)
  seed_mask = mask.copy()

  # 先按有效深度过滤，减少桌面、手和远处背景对位姿估计的干扰。
  valid_depth = ((depth >= zmin) & (depth <= zmax)).astype(np.uint8)
  mask = mask & valid_depth
  depth_in_mask = depth[mask > 0]
  if depth_in_mask.size > 0:
    median_depth = np.median(depth_in_mask)
    mad = np.median(np.abs(depth_in_mask-median_depth))
    band = max(depth_band, depth_mad_scale*mad)
    depth_keep = (depth >= median_depth-band) & (depth <= median_depth+band)
    mask = mask & depth_keep.astype(np.uint8)

  if erode_kernel > 0:
    kernel = np.ones((erode_kernel, erode_kernel), dtype=np.uint8)
    mask = cv2.erode(mask, kernel, iterations=1)
  if open_kernel > 0:
    kernel = np.ones((open_kernel, open_kernel), dtype=np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
  if close_kernel > 0:
    kernel = np.ones((close_kernel, close_kernel), dtype=np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

  num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
  if num_labels <= 1:
    return mask.astype(bool)

  best_id = _select_component(labels, stats, seed_mask, anchor_xy)
  out = (labels == best_id)
  if out.sum() < min_area:
    return mask.astype(bool)
  return out


def _select_component(labels, stats, seed_mask, anchor_xy):
  num_labels = stats.shape[0]
  if anchor_xy is not None:
    ax,ay = np.round(anchor_xy).astype(int)
    if 0 <= ay < labels.shape[0] and 0 <= ax < labels.shape[1] and labels[ay,ax] > 0:
      return labels[ay,ax]

    anchor = np.asarray(anchor_xy, dtype=np.float32)
    scores = []
    seed_area = max(1.0, float(seed_mask.sum()))
    for label_id in range(1, num_labels):
      area = float(stats[label_id, cv2.CC_STAT_AREA])
      cx = stats[label_id, cv2.CC_STAT_LEFT] + stats[label_id, cv2.CC_STAT_WIDTH] * 0.5
      cy = stats[label_id, cv2.CC_STAT_TOP] + stats[label_id, cv2.CC_STAT_HEIGHT] * 0.5
      dist = np.linalg.norm(np.array([cx,cy], dtype=np.float32)-anchor)
      diag = max(1.0, np.linalg.norm([stats[label_id, cv2.CC_STAT_WIDTH], stats[label_id, cv2.CC_STAT_HEIGHT]]))
      anchor_score = max(0.0, 1.0-dist/diag)
      overlap_score = float(((labels == label_id) & (seed_mask > 0)).sum()) / seed_area
      area_score = min(1.0, area / max(1.0, seed_area))
      scores.append((0.45*anchor_score + 0.35*overlap_score + 0.20*area_score, label_id))
    if len(scores) > 0:
      return max(scores, key=lambda x: x[0])[1]

  return 1 + np.argmax(stats[1:, cv2.CC_STAT_AREA])


def mask_foundationpose_inputs(rgb, depth, mask, dilate_kernel=25, zmin=0.10, zmax=2.00,
                               depth_band=None, depth_mad_scale=3.0):
  mask = mask.astype(np.uint8)
  if dilate_kernel > 0:
    kernel = np.ones((dilate_kernel, dilate_kernel), dtype=np.uint8)
    keep = cv2.dilate(mask, kernel, iterations=1).astype(bool)
  else:
    keep = mask.astype(bool)

  if depth_band is not None:
    depth_in_mask = depth[mask.astype(bool)]
    valid = (depth_in_mask >= zmin) & (depth_in_mask <= zmax)
    if valid.sum() > 0:
      median_depth = np.median(depth_in_mask[valid])
      mad = np.median(np.abs(depth_in_mask[valid]-median_depth))
      band = max(depth_band, depth_mad_scale*mad)
      keep = keep & (depth >= median_depth-band) & (depth <= median_depth+band)

  rgb_out = rgb.copy()
  depth_out = depth.copy()
  rgb_out[~keep] = 0
  depth_out[~keep] = 0
  return rgb_out, depth_out


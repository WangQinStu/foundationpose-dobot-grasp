import cv2
import numpy as np
from dataclasses import dataclass, field


@dataclass
class BottleCandidate:
  index: int
  cls_id: int
  conf: float
  bbox: np.ndarray
  mask: np.ndarray
  score: float = 0.0
  score_detail: dict = field(default_factory=dict)


class BottleTargetSelector:
  def __init__(self, target_cls_id=None, min_mask_area=1000, min_valid_depth_ratio=0.15,
               zmin=0.10, zmax=2.00, conf_weight=0.25, area_weight=0.15,
               img_center_weight=0.15, border_weight=0.15, solidity_weight=0.10,
               aspect_weight=0.10, depth_weight=0.10, prefer_large_object=True,
               use_depth=True, crowd_weight=0.20, depth_isolation_weight=0.25,
               isolation_ring_kernel=45, isolation_depth_band=0.08,
               max_interference_ratio=0.22):
    self.target_cls_id = target_cls_id
    self.min_mask_area = min_mask_area
    self.min_valid_depth_ratio = min_valid_depth_ratio
    self.zmin = zmin
    self.zmax = zmax
    self.conf_weight = conf_weight
    self.area_weight = area_weight
    self.img_center_weight = img_center_weight
    self.border_weight = border_weight
    self.solidity_weight = solidity_weight
    self.aspect_weight = aspect_weight
    self.depth_weight = depth_weight
    self.prefer_large_object = prefer_large_object
    self.use_depth = use_depth
    self.crowd_weight = crowd_weight
    self.depth_isolation_weight = depth_isolation_weight
    self.isolation_ring_kernel = isolation_ring_kernel
    self.isolation_depth_band = isolation_depth_band
    self.max_interference_ratio = max_interference_ratio

  def _to_numpy(self, x):
    if x is None:
      return None
    if hasattr(x, 'detach'):
      x = x.detach()
    if hasattr(x, 'cpu'):
      x = x.cpu()
    return np.asarray(x)

  def _mask_from_result(self, result, i, shape):
    H,W = shape[:2]
    if result.masks is None:
      return None
    masks = self._to_numpy(result.masks.data)
    if masks is None or i >= len(masks):
      return None
    mask = masks[i]
    if mask.shape[:2] != (H,W):
      mask = cv2.resize(mask.astype(np.float32), (W,H), interpolation=cv2.INTER_LINEAR)
    return mask > 0.5

  def _make_candidate(self, result, i, image):
    boxes = result.boxes
    bbox = self._to_numpy(boxes.xyxy[i]).astype(np.float32)
    cls_id = int(self._to_numpy(boxes.cls[i]))
    conf = float(self._to_numpy(boxes.conf[i]))
    if self.target_cls_id is not None and cls_id != self.target_cls_id:
      return None
    mask = self._mask_from_result(result, i, image.shape)
    if mask is None:
      return None
    if mask.sum() < self.min_mask_area:
      return None
    return BottleCandidate(index=i, cls_id=cls_id, conf=conf, bbox=bbox, mask=mask)

  def _solidity_score(self, mask):
    contours, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if len(contours)==0:
      return 0.0
    cnt = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(cnt)
    hull = cv2.convexHull(cnt)
    hull_area = cv2.contourArea(hull)
    if hull_area <= 1:
      return 0.0
    return float(np.clip(area / hull_area, 0, 1))

  def _depth_isolation_score(self, mask, depth, median_depth):
    if depth is None or median_depth is None or self.isolation_ring_kernel <= 0:
      return 1.0, 0.0, 0
    kernel = np.ones((self.isolation_ring_kernel, self.isolation_ring_kernel), dtype=np.uint8)
    dilated = cv2.dilate(mask.astype(np.uint8), kernel, iterations=1).astype(bool)
    ring = dilated & (~mask.astype(bool))
    ring_depth = depth[ring]
    valid = (ring_depth >= self.zmin) & (ring_depth <= self.zmax)
    if valid.sum() == 0:
      return 1.0, 0.0, 0

    valid_depth = ring_depth[valid]
    # Objects at a similar depth, or closer than the bottle, are likely to interfere with grasping.
    interference = valid_depth <= median_depth + self.isolation_depth_band
    interference_ratio = float(interference.sum() / max(1, valid.sum()))
    isolation_score = float(np.clip(1.0 - interference_ratio / max(1e-6, self.max_interference_ratio), 0, 1))
    return isolation_score, interference_ratio, int(valid.sum())

  def _score_candidate(self, cand, image, depth=None):
    H,W = image.shape[:2]
    x1,y1,x2,y2 = cand.bbox
    bw = max(1.0, x2-x1)
    bh = max(1.0, y2-y1)
    mask_area = float(cand.mask.sum())
    image_area = float(H*W)

    conf_score = float(np.clip(cand.conf, 0, 1))
    area_ratio = mask_area / max(1.0, image_area)
    if self.prefer_large_object:
      area_score = float(np.clip(area_ratio / 0.18, 0, 1))
    else:
      area_score = float(1.0 - np.clip(area_ratio / 0.18, 0, 1))

    cx = (x1+x2) * 0.5
    cy = (y1+y2) * 0.5
    dist = np.sqrt(((cx-W*0.5)/(W*0.5))**2 + ((cy-H*0.5)/(H*0.5))**2)
    center_score = float(np.clip(1.0 - dist, 0, 1))

    border_margin = min(x1, y1, W-1-x2, H-1-y2)
    border_score = float(np.clip(border_margin / 40.0, 0, 1))

    solidity_score = self._solidity_score(cand.mask)

    aspect = bh / bw
    aspect_score = float(np.exp(-abs(np.log(max(aspect, 1e-6) / 2.6))))

    depth_score = 1.0
    depth_isolation_score = 1.0
    depth_interference_ratio = 0.0
    depth_ring_valid = 0
    median_depth = None
    valid_depth_ratio = None
    if self.use_depth and depth is not None:
      depth_in_mask = depth[cand.mask]
      valid = (depth_in_mask >= self.zmin) & (depth_in_mask <= self.zmax)
      valid_depth_ratio = float(valid.sum() / max(1, depth_in_mask.size))
      if valid.sum() == 0 or valid_depth_ratio < self.min_valid_depth_ratio:
        depth_score = 0.0
      else:
        median_depth = float(np.median(depth_in_mask[valid]))
        depth_score = float(np.clip((self.zmax-median_depth) / max(1e-6, self.zmax-self.zmin), 0, 1))
        depth_isolation_score, depth_interference_ratio, depth_ring_valid = self._depth_isolation_score(cand.mask, depth, median_depth)

    score = (
      self.conf_weight * conf_score +
      self.area_weight * area_score +
      self.img_center_weight * center_score +
      self.border_weight * border_score +
      self.solidity_weight * solidity_score +
      self.aspect_weight * aspect_score +
      self.depth_weight * depth_score
    )

    score = (1.0-self.depth_isolation_weight) * score + self.depth_isolation_weight * depth_isolation_score

    cand.score = float(score)
    cand.score_detail = {
      'final_score': cand.score,
      'conf_score': conf_score,
      'area_score': area_score,
      'area_ratio': area_ratio,
      'center_score': center_score,
      'border_score': border_score,
      'solidity_score': solidity_score,
      'aspect_score': aspect_score,
      'aspect': float(aspect),
      'depth_score': depth_score,
      'depth_isolation_score': depth_isolation_score,
      'depth_interference_ratio': depth_interference_ratio,
      'depth_ring_valid': depth_ring_valid,
      'median_depth': median_depth,
      'valid_depth_ratio': valid_depth_ratio,
    }
    return cand.score

  def _bbox_iou(self, a, b):
    ax1,ay1,ax2,ay2 = a.astype(float)
    bx1,by1,bx2,by2 = b.astype(float)
    ix1,iy1 = max(ax1,bx1), max(ay1,by1)
    ix2,iy2 = min(ax2,bx2), min(ay2,by2)
    iw,ih = max(0.0, ix2-ix1), max(0.0, iy2-iy1)
    inter = iw*ih
    area_a = max(1.0, (ax2-ax1)*(ay2-ay1))
    area_b = max(1.0, (bx2-bx1)*(by2-by1))
    return inter / max(1.0, area_a+area_b-inter)

  def _bbox_center_dist_score(self, a, b):
    ac = np.array([(a[0]+a[2])*0.5, (a[1]+a[3])*0.5], dtype=float)
    bc = np.array([(b[0]+b[2])*0.5, (b[1]+b[3])*0.5], dtype=float)
    diag = max(1.0, np.linalg.norm(a[2:4]-a[0:2]))
    return float(np.clip(1.0 - np.linalg.norm(ac-bc)/diag, 0, 1))

  def _apply_crowd_scores(self, candidates):
    for cand in candidates:
      crowd = 0.0
      for other in candidates:
        if other.index == cand.index:
          continue
        iou = self._bbox_iou(cand.bbox, other.bbox)
        near = self._bbox_center_dist_score(cand.bbox, other.bbox)
        crowd = max(crowd, iou, near*0.6)
      isolation_score = float(np.clip(1.0-crowd, 0, 1))
      base_score = cand.score
      cand.score = float((1.0-self.crowd_weight)*base_score + self.crowd_weight*isolation_score)
      cand.score_detail['base_score'] = base_score
      cand.score_detail['isolation_score'] = isolation_score
      cand.score_detail['crowd_score'] = float(crowd)
      cand.score_detail['final_score'] = cand.score

  def _crop(self, image, bbox, pad=8):
    H,W = image.shape[:2]
    x1,y1,x2,y2 = bbox.astype(int)
    x1 = max(0, x1-pad)
    y1 = max(0, y1-pad)
    x2 = min(W, x2+pad)
    y2 = min(H, y2+pad)
    return image[y1:y2, x1:x2], np.array([x1,y1,x2,y2], dtype=np.int32)

  def make_selection(self, cand, image, depth=None):
    rgb_crop, bbox_xyxy = self._crop(image, cand.bbox)
    x1,y1,x2,y2 = bbox_xyxy
    mask_crop = cand.mask[y1:y2, x1:x2]
    out = {
      'candidate': cand,
      'candidates': [],
      'mask_full': cand.mask.astype(np.uint8),
      'mask_crop': mask_crop.astype(np.uint8),
      'rgb_crop': rgb_crop,
      'bbox_xyxy': bbox_xyxy,
    }
    if depth is not None:
      out['depth_crop'] = depth[y1:y2, x1:x2]
    return out

  def select_best(self, result, image, depth=None):
    if result.boxes is None or len(result.boxes)==0 or result.masks is None:
      return None

    candidates = []
    for i in range(len(result.boxes)):
      cand = self._make_candidate(result, i, image)
      if cand is None:
        continue
      self._score_candidate(cand, image, depth=depth)
      if cand.score_detail.get('depth_score', 1.0) <= 0:
        continue
      candidates.append(cand)

    if len(candidates)==0:
      return None
    self._apply_crowd_scores(candidates)

    out = self.make_selection(max(candidates, key=lambda x: x.score), image, depth=depth)
    out['candidates'] = candidates
    return out

  def visualize(self, image, best):
    vis = image.copy()
    mask = best['mask_full'].astype(bool)
    overlay = np.zeros_like(vis)
    overlay[mask] = (0, 255, 0)
    vis = cv2.addWeighted(vis, 1.0, overlay, 0.35, 0)
    for cand in best.get('candidates', []):
      x1,y1,x2,y2 = cand.bbox.astype(int)
      color = (0, 0, 255) if cand.index == best['candidate'].index else (255, 255, 0)
      cv2.rectangle(vis, (x1,y1), (x2,y2), color, 2)
      cv2.putText(vis, f'{cand.index}:{cand.score:.2f}', (x1, max(20, y1-6)),
                  cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
    return vis

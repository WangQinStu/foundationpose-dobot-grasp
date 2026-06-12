import numpy as np

from yolo_foundationpose.geometry import bbox_center, bbox_iou


class TargetLock:
  def __init__(self, selector, max_lost=30, switch_after_lost=12, min_iou=0.03,
               max_center_ratio=0.65, switch_on_miss=True, follow_yolo_best=False,
               foreground_switch_depth=0.06):
    self.selector = selector
    self.max_lost = max_lost
    self.switch_after_lost = switch_after_lost
    self.min_iou = min_iou
    self.max_center_ratio = max_center_ratio
    self.switch_on_miss = switch_on_miss
    self.follow_yolo_best = follow_yolo_best
    self.foreground_switch_depth = foreground_switch_depth
    self.locked_bbox = None
    self.locked_depth = None
    self.locked_index = None
    self.lost = 0
    self.lock_id = 0

  def reset(self):
    self.locked_bbox = None
    self.locked_depth = None
    self.locked_index = None
    self.lost = 0
    self.lock_id += 1

  def _match_score(self, cand):
    iou = bbox_iou(self.locked_bbox, cand.bbox)
    prev_center = bbox_center(self.locked_bbox)
    cur_center = bbox_center(cand.bbox)
    diag = np.linalg.norm(self.locked_bbox[2:4]-self.locked_bbox[0:2])
    center_dist = np.linalg.norm(cur_center-prev_center) / max(1.0, diag)
    center_score = max(0.0, 1.0-center_dist)

    depth_score = 1.0
    median_depth = cand.score_detail.get('median_depth')
    if self.locked_depth is not None and median_depth is not None:
      depth_score = max(0.0, 1.0-abs(median_depth-self.locked_depth)/0.25)
    return 0.55*iou + 0.30*center_score + 0.15*depth_score, iou, center_dist

  def _start_new_lock(self, selected):
    self.reset()
    cand = selected['candidate']
    self.locked_bbox = cand.bbox.astype(np.float32)
    self.locked_depth = cand.score_detail.get('median_depth')
    self.locked_index = cand.index
    self.lost = 0
    selected['lock_id'] = self.lock_id
    selected['lock_lost'] = self.lost
    selected['lock_switched'] = True
    return selected

  def update(self, raw_best, image, depth=None):
    if raw_best is None:
      self.lost += 1
      if self.lost > self.max_lost:
        self.reset()
      return None

    candidates = raw_best.get('candidates', [])
    if self.locked_bbox is None:
      selected = raw_best
      cand = selected['candidate']
      selected['lock_switched'] = False
    else:
      raw_cand = raw_best['candidate']
      raw_depth = raw_cand.score_detail.get('median_depth')
      if (
          self.foreground_switch_depth > 0
          and self.locked_depth is not None
          and raw_depth is not None
          and raw_cand.index != self.locked_index
          and raw_depth + self.foreground_switch_depth < self.locked_depth):
        return self._start_new_lock(raw_best)

      if self.follow_yolo_best:
        score,iou,center_dist = self._match_score(raw_cand)
        if raw_cand.index != self.locked_index and iou < self.min_iou and center_dist > self.max_center_ratio:
          return self._start_new_lock(raw_best)

      matches = []
      for cand_ in candidates:
        score,iou,center_dist = self._match_score(cand_)
        # 粘性锁定：只要还能在当前 YOLO 结果里找到原目标，就不因为分数跳变而切换。
        if iou >= self.min_iou or center_dist <= self.max_center_ratio:
          matches.append((score, cand_))
      if len(matches)==0:
        self.lost += 1
        if self.switch_on_miss and self.lost >= self.switch_after_lost:
          return self._start_new_lock(raw_best)
        if self.lost > self.max_lost:
          self.reset()
        return None
      cand = max(matches, key=lambda x: x[0])[1]
      selected = self.selector.make_selection(cand, image, depth=depth)
      selected['candidates'] = candidates
      selected['lock_switched'] = False

    # 锁定框只随同一个目标更新，避免多瓶场景下频繁跳目标。
    self.locked_bbox = cand.bbox.astype(np.float32)
    self.locked_depth = cand.score_detail.get('median_depth')
    self.locked_index = cand.index
    self.lost = 0
    selected['lock_id'] = self.lock_id
    selected['lock_lost'] = self.lost
    return selected

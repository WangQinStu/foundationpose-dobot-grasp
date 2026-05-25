import cv2
import numpy as np

from estimater import draw_posed_3d_box, draw_xyz_axis


def colorize_depth(depth, zmax):
  depth_vis = depth.copy()
  depth_vis[depth_vis <= 0] = zmax
  depth_vis = np.clip(depth_vis / zmax * 255, 0, 255).astype(np.uint8)
  return cv2.applyColorMap(255-depth_vis, cv2.COLORMAP_JET)


def draw_score_panel(H, best, pose_ready, fps, refine_info=None):
  panel = np.zeros((H, 520, 3), dtype=np.uint8)
  lines = [f'FPS: {fps:.2f}', f'pose_ready: {pose_ready}']
  if refine_info is not None:
    lines.extend([
      f'sdfr_backend: {refine_info.get("backend", "disabled")}',
      f'sdfr_status: {refine_info.get("status", "idle")}',
    ])
    if refine_info.get('test_perturb_applied', False):
      lines.append('sdfr_test_perturb: True')
    if refine_info.get('raw_output_pose') is not None:
      lines.extend([
        f'sdfr_raw_dR_deg: {refine_info.get("raw_rot_delta_deg", 0.0):.3f}',
        f'sdfr_raw_dT_mm: {refine_info.get("raw_trans_delta_mm", 0.0):.2f}',
      ])
    if refine_info.get('applied', False):
      lines.extend([
        f'sdfr_applied_dR_deg: {refine_info.get("rot_delta_deg", 0.0):.3f}',
        f'sdfr_applied_dT_mm: {refine_info.get("trans_delta_mm", 0.0):.2f}',
      ])
    if refine_info.get('score_before_mm') is not None:
      lines.append(f'sdfr_score_before_mm: {refine_info.get("score_before_mm", 0.0):.2f}')
    if refine_info.get('score_after_mm') is not None:
      lines.append(f'sdfr_score_after_mm: {refine_info.get("score_after_mm", 0.0):.2f}')
    compare = refine_info.get('pose_compare')
    if compare is not None:
      cd = compare.get('candidate_delta', {})
      ud = compare.get('used_delta', {})
      lines.extend([
        'pose compare table:',
        '        dx   dy   dz   dT   dR',
        f'cand {cd.get("dx_mm", 0.0):5.1f} {cd.get("dy_mm", 0.0):5.1f} {cd.get("dz_mm", 0.0):5.1f} {cd.get("dT_mm", 0.0):5.1f} {cd.get("dR_deg", 0.0):4.2f}',
        f'used {ud.get("dx_mm", 0.0):5.1f} {ud.get("dy_mm", 0.0):5.1f} {ud.get("dz_mm", 0.0):5.1f} {ud.get("dT_mm", 0.0):5.1f} {ud.get("dR_deg", 0.0):4.2f}',
      ])
  if best is None:
    lines.append('No valid bottle')
  else:
    cand = best['candidate']
    detail = cand.score_detail
    lines.extend([
      f'lock_id: {best.get("lock_id", 0)}',
      f'selected_id: {cand.index}',
      f'final_score: {detail["final_score"]:.3f}',
      f'conf: {detail["conf_score"]:.3f}',
      f'area: {detail["area_score"]:.3f}',
      f'center: {detail["center_score"]:.3f}',
      f'border: {detail["border_score"]:.3f}',
      f'solidity: {detail["solidity_score"]:.3f}',
      f'aspect: {detail["aspect_score"]:.3f}',
      f'depth: {detail["depth_score"]:.3f}',
      f'depth_iso: {detail.get("depth_isolation_score", 1.0):.3f}',
      f'interf: {detail.get("depth_interference_ratio", 0.0):.3f}',
      f'median_depth: {detail["median_depth"]}',
    ])

  y = 28
  for line in lines:
    color = (0,255,0) if best is not None else (0,0,255)
    cv2.putText(panel, line, (10,y), cv2.FONT_HERSHEY_SIMPLEX, 0.62, color, 2)
    y += 30
  return panel


def draw_pose_overlay(vis_bgr, ob_mask):
  if ob_mask is None:
    return vis_bgr
  pose_overlay = np.zeros_like(vis_bgr)
  pose_overlay[ob_mask.astype(bool)] = (255, 0, 0)
  return cv2.addWeighted(vis_bgr, 1.0, pose_overlay, 0.25, 0)


def draw_pose_axes(vis_bgr, pose, K, to_origin, bbox):
  if pose is None:
    return vis_bgr
  center_pose = pose @ np.linalg.inv(to_origin)
  vis_rgb = cv2.cvtColor(vis_bgr, cv2.COLOR_BGR2RGB)
  vis_rgb = draw_posed_3d_box(K, img=vis_rgb, ob_in_cam=center_pose, bbox=bbox)
  vis_rgb = draw_xyz_axis(vis_rgb, ob_in_cam=center_pose, scale=0.1, K=K,
                          thickness=3, transparency=0, is_input_rgb=True)
  return cv2.cvtColor(vis_rgb, cv2.COLOR_RGB2BGR)


def draw_pose_box(vis_bgr, pose, K, to_origin, bbox, line_color=(0, 200, 255), label=None):
  if pose is None:
    return vis_bgr
  center_pose = pose @ np.linalg.inv(to_origin)
  vis_rgb = cv2.cvtColor(vis_bgr, cv2.COLOR_BGR2RGB)
  rgb_color = tuple(int(c) for c in line_color[::-1])
  vis_rgb = draw_posed_3d_box(K, img=vis_rgb, ob_in_cam=center_pose, bbox=bbox,
                              line_color=rgb_color, linewidth=2)
  vis_bgr = cv2.cvtColor(vis_rgb, cv2.COLOR_RGB2BGR)
  if label:
    cv2.putText(vis_bgr, label, (12, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.9, line_color, 2)
  return vis_bgr


def draw_pose_comparison_views(color_bgr, best, selector, ob_mask, pose_before, pose_after,
                               K, to_origin, bbox):
  before_view = color_bgr.copy()
  after_view = color_bgr.copy()

  if best is not None:
    before_view = selector.visualize(before_view, best)
    after_view = selector.visualize(after_view, best)

  before_view = draw_pose_overlay(before_view, ob_mask)
  after_view = draw_pose_overlay(after_view, ob_mask)

  before_view = draw_pose_box(before_view, pose_before, K, to_origin, bbox,
                              line_color=(0, 200, 255), label='Before SDFR')
  before_view = draw_pose_axes(before_view, pose_before, K, to_origin, bbox)
  after_view = draw_pose_axes(after_view, pose_after, K, to_origin, bbox)
  cv2.putText(after_view, 'After SDFR', (12, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
  return before_view, after_view

import os,time,logging
import cv2
import numpy as np
from ultralytics import YOLO

from estimater import set_logging_format, set_seed
from yolo26.target_selector import BottleTargetSelector
from yolo_foundationpose.camera import create_realsense_pipeline, get_aligned_frame
from yolo_foundationpose.compat import patch_torch_load_for_old_ultralytics
from yolo_foundationpose.dobot_bridge import build_dobot_target_bridge
from yolo_foundationpose.foundation import build_estimator
from yolo_foundationpose.geometry import bbox_center, pose_matches_detection
from yolo_foundationpose.masks import mask_foundationpose_inputs, refine_mask_for_pose
from yolo_foundationpose.pose_refiner import build_pose_refiner
from yolo_foundationpose.ros_pose_publisher import build_ros_pose_publisher
from yolo_foundationpose.target_lock import TargetLock
from yolo_foundationpose.visualization import colorize_depth, draw_pose_axes, draw_pose_overlay, draw_score_panel


WINDOW_NAME = 'D435i YOLO bottle mask -> FoundationPose'


def close_visualization_windows():
  # OpenCV HighGUI on Linux sometimes needs a few event-loop ticks after
  # destroyAllWindows(), otherwise the desktop can mark the last frame frozen.
  try:
    cv2.destroyAllWindows()
    for _ in range(5):
      cv2.waitKey(1)
  except cv2.error as exc:
    logging.warning(f'failed to close OpenCV visualization window cleanly: {exc}')


def create_selector(args):
  # YOLO 可能同时分出多个瓶子。selector 会综合置信度、面积、是否靠近画面中心、
  # 深度是否孤立等因素，选出最适合作为 FoundationPose 输入的目标。
  return BottleTargetSelector(target_cls_id=args.target_cls_id,
                              min_mask_area=args.min_mask_area,
                              zmin=args.zmin,
                              zmax=args.zmax,
                              conf_weight=0.25,
                              area_weight=0.15,
                              img_center_weight=0.15,
                              border_weight=0.15,
                              solidity_weight=0.10,
                              aspect_weight=0.10,
                              depth_weight=0.10,
                              use_depth=True,
                              prefer_large_object=True,
                              depth_isolation_weight=args.depth_isolation_weight,
                              isolation_ring_kernel=args.isolation_ring_kernel,
                              isolation_depth_band=args.isolation_depth_band,
                              max_interference_ratio=args.max_interference_ratio)


def reset_pose(est):
  # FoundationPose 的 track_one 依赖上一帧位姿 est.pose_last。
  # 目标切换、位姿不一致或 mask 失效时必须清掉它，下一次才会重新 register。
  est.pose_last = None
  return None, -100000


def prepare_foundationpose_inputs(args, color_rgb, depth, ob_mask, for_track=False):
  # 可选地把 RGB-D 限制在 YOLO mask 附近。这样可以减少背景深度对
  # register/track 的干扰；关闭相关参数时则把整帧原样交给 FoundationPose。
  use_mask = args.mask_inputs_for_track if for_track else args.mask_inputs_for_register
  if not use_mask:
    return color_rgb, depth

  return mask_foundationpose_inputs(color_rgb, depth, ob_mask,
                                    dilate_kernel=args.input_mask_dilate_kernel,
                                    zmin=args.zmin, zmax=args.zmax,
                                    depth_band=args.input_depth_band,
                                    depth_mad_scale=args.input_depth_mad_scale)


def pose_is_consistent(args, pose, K, best, to_origin, bbox):
  # 用当前 6D pose 投影出的模型位置反查 YOLO 锁定目标，避免跟踪漂移到别的物体。
  # 这里同时检查投影框、深度和 mask/中心位置，阈值来自命令行参数。
  return pose_matches_detection(pose, K, best['candidate'],
                                to_origin=to_origin,
                                bbox=bbox,
                                slack=args.pose_bbox_slack,
                                depth_tolerance=args.pose_depth_tolerance,
                                min_projected_iou=args.pose_min_projected_iou)


def run_foundationpose_step(args, est, K, color_rgb, depth, best, pose, frame_id,
                            last_register_frame, to_origin, bbox, pose_refiner):
  # 单帧位姿更新的核心：
  # - 用 YOLO mask + 深度约束得到更干净的 ob_mask；
  # - 没有历史位姿时调用 register 做初始化；
  # - 有历史位姿时调用 track_one 做快速跟踪；
  # - 如果当前 pose 和 YOLO 锁定目标不一致，就丢弃旧 pose 后重新初始化。
  cand_bbox = best['candidate'].bbox
  anchor_xy = bbox_center(cand_bbox)
  refine_info = pose_refiner.empty_info()
  ob_mask = refine_mask_for_pose(best['mask_full'], depth, args.zmin, args.zmax,
                                 open_kernel=args.mask_open_kernel,
                                 close_kernel=args.mask_close_kernel,
                                 erode_kernel=args.mask_erode_kernel,
                                 depth_band=args.mask_depth_band,
                                 depth_mad_scale=args.mask_depth_mad_scale,
                                 min_area=args.min_pose_mask_area,
                                 anchor_xy=anchor_xy)

  if ob_mask.sum() < args.min_pose_mask_area:
    logging.info('pose mask too small after refinement, skip FoundationPose this frame')
    pose, last_register_frame = reset_pose(est)
    refine_info['status'] = 'mask_too_small'
    return pose, ob_mask, last_register_frame, refine_info

  if pose is not None and not pose_is_consistent(args, pose, K, best, to_origin, bbox):
    logging.info('previous pose does not match current YOLO target, re-register')
    pose, last_register_frame = reset_pose(est)

  need_register = pose is None or args.register_every_frame
  if need_register and frame_id-last_register_frame >= args.register_retry_interval:
    # 首帧或丢失后重新注册；注册比跟踪慢，所以用 retry interval 限制频率。
    fp_rgb, fp_depth = prepare_foundationpose_inputs(args, color_rgb, depth, ob_mask, for_track=False)
    pose = est.register(K=K.astype(np.float32), rgb=fp_rgb, depth=fp_depth, ob_mask=ob_mask,
                        iteration=args.est_refine_iter)
    last_register_frame = frame_id
  elif pose is not None:
    # 正常跟踪路径：复用上一帧 pose_last，FoundationPose 只做少量 refinement。
    fp_rgb, fp_depth = prepare_foundationpose_inputs(args, color_rgb, depth, ob_mask, for_track=True)
    pose = est.track_one(rgb=fp_rgb, depth=fp_depth, K=K.astype(np.float32),
                         iteration=args.track_refine_iter)

  if pose is not None:
    pose, refine_info = pose_refiner.refine(color_rgb, depth, ob_mask, pose, K, args.mesh_file)

  return pose, ob_mask, last_register_frame, refine_info


def update_target(args, yolo, selector, target_lock, color_bgr, depth, frame_id):
  # YOLO 可以按间隔帧运行，降低整体延迟。没运行 YOLO 的帧返回 yolo_ran=False，
  # 主循环会决定是否只用 FoundationPose 在两次检测之间继续跟踪。
  if frame_id % max(1, args.yolo_interval) != 0:
    return None, False

  results = yolo.predict(source=color_bgr, imgsz=args.imgsz, conf=args.conf,
                         device=args.yolo_device, verbose=False)
  raw_best = selector.select_best(results[0], color_bgr, depth=depth)
  # target_lock 给目标选择加“粘性”：短暂漏检或分数抖动时尽量保持同一个瓶子。
  return target_lock.update(raw_best, color_bgr, depth=depth), True


def run(args):
  # 主流程数据流：
  # RealSense RGB-D -> YOLO 分割/目标锁定 -> FoundationPose register/track
  # -> 可视化/保存 4x4 物体位姿矩阵。
  set_logging_format()
  set_seed(0)
  os.makedirs(f'{args.debug_dir}/ob_in_cam', exist_ok=True)

  logging.info('loading YOLO model')
  patch_torch_load_for_old_ultralytics()
  yolo = YOLO(args.yolo_weights)

  logging.info('initializing FoundationPose')
  est, to_origin, bbox = build_estimator(args.mesh_file, args.debug_dir, args.debug)
  pose_refiner = build_pose_refiner(args, est)

  selector = create_selector(args)
  target_lock = TargetLock(selector,
                           max_lost=args.lock_max_lost,
                           switch_after_lost=args.lock_switch_after_lost,
                           min_iou=args.lock_min_iou,
                           max_center_ratio=args.lock_max_center_ratio,
                           switch_on_miss=bool(args.lock_switch_on_miss),
                           follow_yolo_best=bool(args.follow_yolo_best))

  ros_pose_publisher = build_ros_pose_publisher(args)
  dobot_bridge = build_dobot_target_bridge(args)

  logging.info('starting RealSense D435i')
  pipeline, align, depth_scale, K = create_realsense_pipeline(args.width, args.height, args.fps)
  K = np.asarray(K, dtype=np.float32)
  logging.info(f'RealSense depth_scale:{depth_scale}')
  logging.info(f'K:\n{K}')

  pose = None
  best = None
  ob_mask = None
  active_lock_id = None
  frame_id = 0
  # last_register_frame 用来避免 mask 很差或目标丢失时每帧都触发昂贵的 register。
  last_register_frame = -100000
  last_time = time.time()
  fps_avg = 0.0
  last_refine_info = pose_refiner.empty_info()

  try:
    try:
      cv2.startWindowThread()
    except cv2.error:
      pass
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)

    while True:
      # color_bgr 用于 OpenCV/YOLO 显示；color_rgb 才是 FoundationPose 需要的输入。
      # depth 已经对齐到彩色图，并且超出 zmin/zmax 的值会被置 0。
      color_bgr, color_rgb, depth = get_aligned_frame(pipeline, align, depth_scale, args.zmin, args.zmax)
      if color_bgr is None:
        continue

      new_best, yolo_ran = update_target(args, yolo, selector, target_lock, color_bgr, depth, frame_id)
      if yolo_ran:
        best = new_best
        if best is not None:
          best['frame_id'] = frame_id

      if best is not None:
        lock_changed = active_lock_id is not None and best.get('lock_id') != active_lock_id
        if lock_changed or best.get('lock_switched', False):
          # 锁定目标变了，上一帧 pose 属于旧物体，不能继续 track。
          logging.info('YOLO locked target changed, reset FoundationPose for new target')
          pose, last_register_frame = reset_pose(est)
          last_refine_info = pose_refiner.empty_info()
          last_refine_info['status'] = 'reset_target_changed'
          best['lock_switched'] = False
        active_lock_id = best.get('lock_id')
      elif active_lock_id is not None and target_lock.locked_bbox is None:
        logging.info('YOLO target lock released, reset FoundationPose')
        pose, last_register_frame = reset_pose(est)
        active_lock_id = None
        last_refine_info = pose_refiner.empty_info()
        last_refine_info['status'] = 'reset_target_released'
      elif best is None and pose is not None:
        # 原锁定目标短暂没被 YOLO 分到时先保持 pose，不马上切到别的瓶子。
        ob_mask = None

      fresh_yolo_mask = best is not None and best.get('frame_id') == frame_id
      if best is not None and fresh_yolo_mask:
        # 当前帧有新鲜 YOLO mask，优先用它约束 FoundationPose。
        pose, ob_mask, last_register_frame, last_refine_info = run_foundationpose_step(
          args, est, K, color_rgb, depth, best, pose, frame_id, last_register_frame,
          to_origin, bbox, pose_refiner)

        if pose is not None and not pose_is_consistent(args, pose, K, best, to_origin, bbox):
          logging.info('pose is inconsistent with locked YOLO target, reset FoundationPose')
          pose, last_register_frame = reset_pose(est)
          last_refine_info = pose_refiner.empty_info()
          last_refine_info['status'] = 'reset_pose_mismatch'
      elif best is not None and pose is not None and args.track_between_yolo:
        # YOLO 间隔帧不要复用旧 mask；旧分割会把 track_one 拉回上一帧目标区域。
        pose = est.track_one(rgb=color_rgb, depth=depth, K=K.astype(np.float32),
                             iteration=args.track_refine_iter)
        ob_mask = None
        last_refine_info = pose_refiner.empty_info()
        last_refine_info['status'] = 'track_only'
      else:
        ob_mask = None
        last_refine_info = pose_refiner.empty_info()
        if best is None:
          last_refine_info['status'] = 'no_target'

      now = time.time()
      fps = 1.0 / max(1e-6, now-last_time)
      fps_avg = 0.9*fps_avg + 0.1*fps if fps_avg > 0 else fps
      last_time = now

      vis_bgr = color_bgr.copy()
      if best is not None:
        vis_bgr = selector.visualize(vis_bgr, best)
      # ob_mask 只在当前帧 YOLO 参与更新时显示；pose 坐标轴来自 FoundationPose。
      vis_bgr = draw_pose_overlay(vis_bgr, ob_mask)
      vis_bgr = draw_pose_axes(vis_bgr, pose, K, to_origin, bbox)

      if pose is not None and args.save_pose:
        # 保存的是物体坐标系到相机坐标系的 4x4 矩阵，后续接机器人时通常还要
        # 乘以相机到机器人基座的外参。
        np.savetxt(f'{args.debug_dir}/ob_in_cam/{frame_id:06d}.txt', pose.reshape(4,4))
      if pose is not None and ros_pose_publisher is not None:
        ros_pose_publisher.publish(pose)
      if pose is not None and dobot_bridge is not None:
        dobot_bridge.publish_if_ready(pose)

      depth_vis = colorize_depth(depth, args.zmax)
      panel = draw_score_panel(color_bgr.shape[0], best, pose is not None, fps_avg,
                               refine_info=last_refine_info)
      show = np.hstack([vis_bgr, depth_vis, panel])
      cv2.imshow(WINDOW_NAME, show)

      key = cv2.waitKey(1) & 0xFF
      if key == ord('q') or key == 27:
        break
      if key == ord('r'):
        # r：清空 FoundationPose 和 YOLO 锁定状态，从头选择并注册目标。
        pose, last_register_frame = reset_pose(est)
        target_lock.reset()
        active_lock_id = None
        best = None
        ob_mask = None
        last_refine_info = pose_refiner.empty_info()
        last_refine_info['status'] = 'manual_reset'
        logging.info('pose reset')
      if key == ord('n'):
        # n：释放当前锁定目标；下一次 YOLO 会重新选择最优目标。
        pose, last_register_frame = reset_pose(est)
        target_lock.reset()
        active_lock_id = None
        best = None
        ob_mask = None
        last_refine_info = pose_refiner.empty_info()
        last_refine_info['status'] = 'manual_release'
        logging.info('target lock released')

      frame_id += 1
  except KeyboardInterrupt:
    logging.info('received Ctrl+C, closing visualization and exiting')
  finally:
    close_visualization_windows()
    if dobot_bridge is not None:
      dobot_bridge.close()
    if ros_pose_publisher is not None:
      ros_pose_publisher.close()
    try:
      pipeline.stop()
    finally:
      close_visualization_windows()

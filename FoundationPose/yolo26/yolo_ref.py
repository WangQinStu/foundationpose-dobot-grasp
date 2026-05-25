import os
import cv2
import time
import argparse
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO

from target_selector import BottleTargetSelector


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--weights",
        type=str,
        required=True,
        help="YOLO segmentation model path, e.g. best.pt"
    )
    parser.add_argument(
        "--imgsz",
        type=int,
        default=640,
        help="YOLO inference size"
    )
    parser.add_argument(
        "--conf",
        type=float,
        default=0.35,
        help="YOLO confidence threshold"
    )
    parser.add_argument(
        "--device",
        type=str,
        default="0",
        help="cuda device, e.g. 0 or cpu"
    )
    parser.add_argument(
        "--save_dir",
        type=str,
        default="./selector_output",
        help="directory to save selected target data when pressing 's'"
    )
    parser.add_argument(
        "--target_cls_id",
        type=int,
        default=None,
        help="target class id, set None if only one class"
    )
    parser.add_argument(
        "--fps_avg_len",
        type=int,
        default=30,
        help="number of frames for FPS moving average"
    )
    return parser.parse_args()


def create_realsense_pipeline():
    pipeline = rs.pipeline()
    config = rs.config()

    # D435i 常用配置
    config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)

    profile = pipeline.start(config)

    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()  # 通常是米/单位
    print(f"[INFO] depth_scale = {depth_scale}")

    align = rs.align(rs.stream.color)

    color_stream = profile.get_stream(rs.stream.color).as_video_stream_profile()
    intr = color_stream.get_intrinsics()
    K = np.array([
        [intr.fx, 0, intr.ppx],
        [0, intr.fy, intr.ppy],
        [0, 0, 1]
    ], dtype=np.float32)

    return pipeline, align, depth_scale, K


def get_aligned_frames(pipeline, align):
    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)

    color_frame = aligned_frames.get_color_frame()
    depth_frame = aligned_frames.get_depth_frame()

    if not color_frame or not depth_frame:
        return None, None

    color = np.asanyarray(color_frame.get_data())          # BGR
    depth = np.asanyarray(depth_frame.get_data())          # uint16

    return color, depth


def colorize_depth(depth, max_mm=1500):
    """
    仅用于显示
    """
    depth_vis = depth.copy().astype(np.float32)
    depth_vis[depth_vis <= 0] = max_mm
    depth_vis = np.clip(depth_vis, 0, max_mm)
    depth_vis = (depth_vis / max_mm * 255).astype(np.uint8)
    depth_vis = cv2.applyColorMap(255 - depth_vis, cv2.COLORMAP_JET)
    return depth_vis


def draw_selected_info(panel, best_dict):
    cand = best_dict["candidate"]
    detail = cand.score_detail

    lines = [
        f"selected_id: {cand.index}",
        f"final_score: {detail['final_score']:.3f}",
        f"conf:        {detail['conf_score']:.3f}",
        f"area:        {detail['area_score']:.3f}",
        f"center:      {detail['center_score']:.3f}",
        f"border:      {detail['border_score']:.3f}",
        f"solidity:    {detail['solidity_score']:.3f}",
        f"aspect:      {detail['aspect_score']:.3f}",
        f"depth:       {detail['depth_score']:.3f}",
        f"median_depth:{detail['median_depth']}",
    ]

    y = 25
    for line in lines:
        cv2.putText(panel, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        y += 28

    return panel


def save_selected_data(save_dir, color, depth, best_dict, K):
    os.makedirs(save_dir, exist_ok=True)
    ts = time.strftime("%Y%m%d_%H%M%S")

    rgb_full_path = os.path.join(save_dir, f"{ts}_rgb_full.png")
    depth_full_path = os.path.join(save_dir, f"{ts}_depth_full.png")
    mask_full_path = os.path.join(save_dir, f"{ts}_mask_full.png")
    crop_rgb_path = os.path.join(save_dir, f"{ts}_rgb_crop.png")
    crop_depth_path = os.path.join(save_dir, f"{ts}_depth_crop.png")
    crop_mask_path = os.path.join(save_dir, f"{ts}_mask_crop.png")
    info_path = os.path.join(save_dir, f"{ts}_info.txt")
    K_path = os.path.join(save_dir, f"{ts}_cam_K.txt")

    cv2.imwrite(rgb_full_path, color)
    cv2.imwrite(depth_full_path, depth)
    cv2.imwrite(mask_full_path, (best_dict["mask_full"] * 255).astype(np.uint8))
    cv2.imwrite(crop_rgb_path, best_dict["rgb_crop"])
    cv2.imwrite(crop_mask_path, (best_dict["mask_crop"] * 255).astype(np.uint8))

    if "depth_crop" in best_dict:
        cv2.imwrite(crop_depth_path, best_dict["depth_crop"])

    np.savetxt(K_path, K, fmt="%.8e")

    cand = best_dict["candidate"]
    with open(info_path, "w", encoding="utf-8") as f:
        f.write(f"bbox_xyxy_expand = {best_dict['bbox_xyxy'].tolist()}\n")
        f.write(f"origin_bbox_xyxy = {cand.bbox.tolist()}\n")
        f.write(f"selected_index = {cand.index}\n")
        f.write(f"class_id = {cand.cls_id}\n")
        f.write(f"score = {cand.score}\n")
        f.write(f"score_detail = {cand.score_detail}\n")

    print(f"[INFO] saved selected target to: {save_dir}")


def main():
    args = parse_args()
    os.makedirs(args.save_dir, exist_ok=True)

    print("[INFO] loading YOLO model...")
    model = YOLO(args.weights)

    print("[INFO] starting RealSense...")
    pipeline, align, depth_scale, K = create_realsense_pipeline()

    selector = BottleTargetSelector(
        target_cls_id=args.target_cls_id,
        min_mask_area=1000,
        conf_weight=0.25,
        area_weight=0.15,
        img_center_weight=0.15,
        border_weight=0.15,
        solidity_weight=0.10,
        aspect_weight=0.10,
        depth_weight=0.10,
        use_depth=True,
        prefer_large_object=True,
    )

    fps_buffer = []
    prev_time = time.time()

    print("[INFO] Press 'q' to quit")
    print("[INFO] Press 's' to save current selected target")
    print("[INFO] Press 'p' to pause/unpause")

    paused = False
    freeze_color = None
    freeze_depth = None
    freeze_best = None
    freeze_vis = None

    try:
        while True:
            if not paused:
                color, depth = get_aligned_frames(pipeline, align)
                if color is None or depth is None:
                    continue

                # YOLO 推理
                results = model.predict(
                    source=color,
                    imgsz=args.imgsz,
                    conf=args.conf,
                    device=args.device,
                    verbose=False
                )
                result = results[0]

                best = selector.select_best(result, color, depth=depth)

                vis_color = color.copy()

                if best is not None:
                    vis_color = selector.visualize(vis_color, best)

                    x1, y1, x2, y2 = best["bbox_xyxy"]
                    cv2.rectangle(vis_color, (x1, y1), (x2, y2), (0, 0, 255), 2)

                    cand = best["candidate"]
                    label = f"SELECTED id={cand.index} score={cand.score:.3f}"
                    cv2.putText(
                        vis_color, label, (x1, max(25, y1 - 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2
                    )

                depth_vis = colorize_depth(depth, max_mm=1500)

                # 右侧文字面板
                panel = np.zeros((color.shape[0], 420, 3), dtype=np.uint8)
                if best is not None:
                    panel = draw_selected_info(panel, best)
                else:
                    cv2.putText(panel, "No valid target", (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

                # FPS
                now = time.time()
                fps = 1.0 / max(1e-6, now - prev_time)
                prev_time = now
                fps_buffer.append(fps)
                if len(fps_buffer) > args.fps_avg_len:
                    fps_buffer.pop(0)
                fps_avg = sum(fps_buffer) / len(fps_buffer)

                cv2.putText(
                    vis_color, f"FPS: {fps_avg:.2f}", (20, 35),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0), 2
                )

                top = np.hstack([vis_color, depth_vis])
                show = np.hstack([top, panel])

                freeze_color = color.copy()
                freeze_depth = depth.copy()
                freeze_best = best
                freeze_vis = show.copy()

            else:
                show = freeze_vis if freeze_vis is not None else np.zeros((480, 1200, 3), dtype=np.uint8)
                cv2.putText(show, "PAUSED", (30, 70),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 255), 3)

            cv2.imshow("D435i | YOLO-seg | Target Selector", show)
            key = cv2.waitKey(1) & 0xFF

            if key == ord('q'):
                break
            elif key == ord('p'):
                paused = not paused
            elif key == ord('s'):
                if freeze_color is not None and freeze_depth is not None and freeze_best is not None:
                    save_selected_data(args.save_dir, freeze_color, freeze_depth, freeze_best, K)
                else:
                    print("[WARN] nothing to save")

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
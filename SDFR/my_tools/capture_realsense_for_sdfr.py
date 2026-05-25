import pyrealsense2 as rs
import numpy as np
import cv2
import os
from pathlib import Path

# ====================== 配置参数 ======================
ROOT_DIR = "/home/qin-desktop-5060/workspace/SDFR/datasets"  # 你的数据集根目录
OBJ_ID = 1  # 物体ID（对应lm模型）
DEPTH_CLOSE = 200  # 最近深度(mm)
DEPTH_FAR = 1500  # 最远深度(mm)，只保留物体
WIDTH = 640
HEIGHT = 480
FPS = 30
# ======================================================

# 自动创建文件夹
rgb_dir = Path(ROOT_DIR) / "data" / "rgb"
depth_dir = Path(ROOT_DIR) / "data" / "depth"
mask_dir = Path(ROOT_DIR) / "data" / "mask"

rgb_dir.mkdir(parents=True, exist_ok=True)
depth_dir.mkdir(parents=True, exist_ok=True)
mask_dir.mkdir(parents=True, exist_ok=True)

# 配置Realsense
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.bgr8, FPS)
config.enable_stream(rs.stream.depth, WIDTH, HEIGHT, rs.format.z16, FPS)

# 启动
pipeline.start(config)
align = rs.align(rs.stream.color)

print("=" * 60)
print("按 s 保存一张图")
print("按 q 退出")
print("=" * 60)

img_idx = 1

try:
    while True:
        # 等待帧并对齐
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)

        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()

        if not color_frame or not depth_frame:
            continue

        # 转numpy
        color = np.asanyarray(color_frame.get_data())
        depth = np.asanyarray(depth_frame.get_data())

        # 生成mask：深度在[DEPTH_CLOSE, DEPTH_FAR]之间为物体(255)
        mask = np.zeros_like(depth, dtype=np.uint8)
        mask[(depth > DEPTH_CLOSE) & (depth < DEPTH_FAR)] = 255

        # 深度上色显示
        depth_colormap = cv2.applyColorMap(
            cv2.convertScaleAbs(depth, alpha=0.03), cv2.COLORMAP_JET
        )

        # 拼接显示
        show = np.hstack((color, depth_colormap, cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)))
        cv2.imshow("RGB | Depth | Mask", show)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            # 6位文件名
            fname = f"{img_idx:06d}.png"

            cv2.imwrite(str(rgb_dir / fname), color)
            cv2.imwrite(str(depth_dir / fname), depth)
            cv2.imwrite(str(mask_dir / fname), mask)

            print(f"已保存: {fname}")
            img_idx += 1

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
import pyrealsense2 as rs
import numpy as np
import cv2
import os

# ===================== 可修改配置 =====================
COLOR_OUTPUT_PATH = "d435_color.mp4"
DEPTH_OUTPUT_PATH = "d435_depth.mp4"
WIDTH = 848
HEIGHT = 480
FPS = 30
# ======================================================

# 1. 相机核心组件初始化（循环外仅执行1次）
pipeline = rs.pipeline()
config = rs.config()
align = rs.align(rs.stream.color)  # 深度帧与彩色帧像素对齐，解决画面错位
colorizer = rs.colorizer()  # 深度图着色器，实现可视化

# 2. 流配置
config.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.bgr8, FPS)
config.enable_stream(rs.stream.depth, WIDTH, HEIGHT, rs.format.z16, FPS)

# 3. 视频写入器初始化（循环外仅执行1次）
color_fourcc = cv2.VideoWriter_fourcc(*'mp4v')
color_writer = cv2.VideoWriter(COLOR_OUTPUT_PATH, color_fourcc, FPS, (WIDTH, HEIGHT))
depth_fourcc = cv2.VideoWriter_fourcc(*'mp4v')
depth_writer = cv2.VideoWriter(DEPTH_OUTPUT_PATH, depth_fourcc, FPS, (WIDTH, HEIGHT))

# 4. 预览窗口初始化（循环外仅执行1次，彻底解决弹框）
WINDOW_NAME = "D435 彩色+深度采集预览"
cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_AUTOSIZE)

# 启动提示
print("=== 采集已启动 ===")
print(f"彩色视频: {COLOR_OUTPUT_PATH}")
print(f"深度视频: {DEPTH_OUTPUT_PATH}")
print("按键盘 q 键 或 点击窗口×关闭按钮 停止采集")

# 核心采集逻辑
try:
    # 启动相机流
    pipeline.start(config)
    print("相机启动成功，正在采集...")

    while True:
        # 等待帧数据，5秒超时防止卡死
        frames = pipeline.wait_for_frames(timeout_ms=5000)
        # 帧对齐：让深度和彩色画面像素一一对应
        aligned_frames = align.process(frames)

        # 获取对齐后的有效帧
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()
        if not color_frame or not depth_frame:
            continue

        # 格式转换
        color_image = np.asanyarray(color_frame.get_data())
        depth_colormap = np.asanyarray(colorizer.colorize(depth_frame).get_data())

        # 写入视频文件
        color_writer.write(color_image)
        depth_writer.write(depth_colormap)

        # 左右拼接预览
        preview_image = np.hstack((color_image, depth_colormap))
        cv2.imshow(WINDOW_NAME, preview_image)

        # 双退出逻辑
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q') or cv2.getWindowProperty(WINDOW_NAME, cv2.WND_PROP_VISIBLE) < 1:
            print("收到停止指令，正在结束采集...")
            break

except Exception as e:
    print(f"采集异常: {str(e)}")

finally:
    # 安全释放所有资源
    pipeline.stop()
    color_writer.release()
    depth_writer.release()
    cv2.destroyAllWindows()
    print("=== 采集完成，所有资源已释放 ===")
    print(f"彩色视频: {COLOR_OUTPUT_PATH}")
    print(f"深度视频: {DEPTH_OUTPUT_PATH}")
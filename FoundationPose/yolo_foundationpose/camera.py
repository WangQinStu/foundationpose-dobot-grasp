import cv2
import numpy as np
import pyrealsense2 as rs


def _start_pipeline_with_profile(pipeline, width, height, fps):
  config = rs.config()
  config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
  config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
  return pipeline.start(config)


def create_realsense_pipeline(width, height, fps):
  # 同时开启彩色和深度流，后面会把深度对齐到彩色图坐标系。
  pipeline = rs.pipeline()
  candidates = [
    (width, height, fps),
    (848, 480, 10),
    (640, 480, 30),
    (640, 480, 15),
    (480, 270, 30),
    (480, 270, 15),
    (424, 240, 30),
  ]

  tried = []
  last_error = None
  profile = None
  for candidate in candidates:
    if candidate in tried:
      continue
    tried.append(candidate)
    try:
      profile = _start_pipeline_with_profile(pipeline, *candidate)
      if candidate != (width, height, fps):
        print(f'[create_realsense_pipeline()] requested {width}x{height}@{fps} not available, '
              f'using {candidate[0]}x{candidate[1]}@{candidate[2]}')
      break
    except RuntimeError as exc:
      last_error = exc

  if profile is None:
    tried_text = ', '.join(f'{w}x{h}@{f}' for w, h, f in tried)
    raise RuntimeError(f'Could not start RealSense stream. Tried: {tried_text}') from last_error

  depth_sensor = profile.get_device().first_depth_sensor()
  depth_scale = depth_sensor.get_depth_scale()
  align = rs.align(rs.stream.color)

  # FoundationPose 需要相机内参 K，把 RealSense intrinsics 转成常见的 3x3 矩阵。
  color_stream = profile.get_stream(rs.stream.color).as_video_stream_profile()
  intr = color_stream.get_intrinsics()
  K = np.array([[intr.fx, 0, intr.ppx],
                [0, intr.fy, intr.ppy],
                [0, 0, 1]], dtype=np.float32)
  return pipeline, align, depth_scale, K


def get_aligned_frame(pipeline, align, depth_scale, zmin, zmax):
  # wait_for_frames 是阻塞调用；返回后把深度帧对齐到彩色帧，保证 RGB、depth、
  # mask 和相机内参 K 都在同一个像素坐标系下。
  frames = pipeline.wait_for_frames()
  frames = align.process(frames)
  color_frame = frames.get_color_frame()
  depth_frame = frames.get_depth_frame()
  if not color_frame or not depth_frame:
    return None, None, None

  color_bgr = np.asanyarray(color_frame.get_data())
  depth_raw = np.asanyarray(depth_frame.get_data())
  depth = depth_raw.astype(np.float32) * depth_scale
  # 这里的单位是米。无效/太近/太远的深度置 0，后续 mask 和 FoundationPose 会忽略。
  depth[(depth < zmin) | (depth > zmax)] = 0

  # FoundationPose 使用 RGB 输入，OpenCV 和 RealSense 预览保留 BGR。
  color_rgb = cv2.cvtColor(color_bgr, cv2.COLOR_BGR2RGB)
  return color_bgr, color_rgb, depth

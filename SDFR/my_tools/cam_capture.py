import pyrealsense2 as rs
import numpy as np
import cv2
import open3d as o3d
import os

# ===================== 【配置】自动保存路径 =====================
save_dir = "/home/qin-desktop-5060/workspace/SDFR/my_tools/real_data"
os.makedirs(save_dir, exist_ok=True)
os.makedirs(os.path.join(save_dir, "rgb"), exist_ok=True)
os.makedirs(os.path.join(save_dir, "depth"), exist_ok=True)
os.makedirs(os.path.join(save_dir, "label"), exist_ok=True)

# 自动从 000000 开始编号（不会覆盖）
def get_next_index(save_dir):
    label_dir = os.path.join(save_dir, "label")
    files = os.listdir(label_dir)
    max_idx = 0
    for f in files:
        if f.endswith(".npz"):
            try:
                idx = int(f.split(".")[0])
                max_idx = max(max_idx, idx)
            except:
                pass
    return max_idx + 1

# ===================== 1. 初始化 RealSense D435 =====================
pipeline = rs.pipeline()
config = rs.config()
# 用 D435 最稳的分辨率
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
profile = pipeline.start(config)

# ✅ 修复3：获取【彩色流】内参（投影到彩色图必须用彩色内参）
color_profile = profile.get_stream(rs.stream.color).as_video_stream_profile()
intr_color = color_profile.get_intrinsics()
fx, fy = intr_color.fx, intr_color.fy
cx, cy = intr_color.ppx, intr_color.ppy
K = np.array([[fx, 0, cx],
              [0, fy, cy],
              [0,  0, 1]], dtype=np.float32)
np.save("../my_tools/real_data/K.npy", K)
print("✅ 彩色相机内参 K.npy 已保存")

# ===================== 2. 采集对齐图像（每帧对齐，稳定） =====================
align = rs.align(rs.stream.color)
# 预热30帧，让曝光稳定
for _ in range(30):
    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)  # ✅ 修复2：循环内每帧都对齐

# 取最终稳定帧
frames = pipeline.wait_for_frames()
aligned_frames = align.process(frames)
depth_frame = aligned_frames.get_depth_frame()
color_frame = aligned_frames.get_color_frame()

color_image = np.asanyarray(color_frame.get_data())
depth_image = np.asanyarray(depth_frame.get_data())

# ===================== 3. 生成点云（顺序正确） =====================
pc = rs.pointcloud()
points_rs = pc.calculate(depth_frame)
pc.map_to(color_frame)  # ✅ 修复4：必须在calculate之后

v = np.asanyarray(points_rs.get_vertices())
points = v.view((np.float32, 3))  # 形状: (N, 3) [X, Y, Z]，相机坐标系
colors = np.asanyarray(color_frame.get_data()).reshape(-1, 3) / 255.0

# ===================== 4. 距离过滤 =====================
mask_depth = (points[:, 2] > 0.1) & (points[:, 2] < 1.5)
points = points[mask_depth]
colors = colors[mask_depth]

# ===================== 5. 框选目标（核心修复：投影函数） =====================
print("👉 框选目标物体，按 ENTER 确认")
roi = cv2.selectROI("select object", color_image)
x1, y1, w, h = roi
x2, y2 = x1 + w, y1 + h

# ✅ 修复1：正确的3D→2D投影（处理Y轴符号，相机Y向下）
def project_3d_to_2d(pt, K):
    X, Y, Z = pt
    if Z < 1e-3:
        return -1, -1
    # 相机坐标系: X右, Y下, Z前 → 直接对应OpenCV图像坐标系
    u = (X * K[0,0] / Z) + K[0,2]
    v = (Y * K[1,1] / Z) + K[1,2]
    return int(round(u)), int(round(v))

mask_obj = []
for p in points:
    u, v = project_3d_to_2d(p, K)
    # 检查是否在ROI内
    if x1 < u < x2 and y1 < v < y2:
        mask_obj.append(True)
    else:
        mask_obj.append(False)

object_points = points[mask_obj]
object_colors = colors[mask_obj]

# 去噪
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(object_points)
pcd.colors = o3d.utility.Vector3dVector(object_colors)
pcd, ind = pcd.remove_statistical_outlier(nb_neighbors=30, std_ratio=1.0)
final_points = np.asarray(pcd.points)

# ===================== 6. 保存 =====================
idx = get_next_index(save_dir)
filename = f"{idx:06d}"

rgb_save_path = os.path.join(save_dir, "rgb", f"{filename}.png")
cv2.imwrite(rgb_save_path, color_image)

depth_save_path = os.path.join(save_dir, "depth", f"{filename}.png")
cv2.imwrite(depth_save_path, depth_image)

q_gt = np.array([0, 0, 0, 1], dtype=np.float32)
t_gt = np.array([0, 0, 0], dtype=np.float32)
npz_path = os.path.join(save_dir, "label", f"{filename}.npz")
np.savez(npz_path, points=final_points, q_gt=q_gt, t_gt=t_gt)

ply_path = os.path.join(save_dir, f"{filename}.ply")
o3d.io.write_point_cloud(ply_path, pcd)

print(f"\n📁 保存成功：")
print(f"RGB: {rgb_save_path}")
print(f"Depth: {depth_save_path}")
print(f"NPZ: {npz_path}")
print(f"PLY: {ply_path}")
print(f"✅ 目标点云数量：{len(final_points)}")

# 显示
o3d.visualization.draw_geometries([pcd], window_name="目标彩色点云")

pipeline.stop()
cv2.destroyAllWindows()
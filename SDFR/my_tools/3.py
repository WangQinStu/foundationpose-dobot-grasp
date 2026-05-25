import open3d as o3d
import numpy as np
import torch
from lib import rendering

# ====================== 只改这里 ======================
OBJ_FILE      = "/home/qin-desktop-5060/workspace/SDFR/datasets/models/lm/obj_000001.obj"
TARGET_PLY    = "/home/qin-desktop-5060/workspace/SDFR/datasets/render/standard/lm_obj_000001/R_50_70_t_0.1_0.2/target/model_0.ply"
GT_POSE_TXT   = "/home/qin-desktop-5060/workspace/SDFR/datasets/render/standard/lm_obj_000001/R_50_70_t_0.1_0.2/gt_pose/0.txt"
SCALE_TXT     = OBJ_FILE.replace('.obj', '_scale.txt')
# ======================================================

print("🔍 最终版坐标系诊断（修复平移偏移）...")

# 1. 读取所有数据
mesh_o3d = o3d.io.read_triangle_mesh(OBJ_FILE)
pcd_model_raw = mesh_o3d.sample_points_uniformly(10000)
pcd_target_scene = o3d.io.read_point_cloud(TARGET_PLY)

# 🔥 修复1：统一单位，目标点云放大1000倍
pcd_target_scene.scale(1000.0, center=(0,0,0))

# 读取GT位姿和缩放信息
gt_pose = np.loadtxt(GT_POSE_TXT)
obj_scale, cx, cy, cz = np.loadtxt(SCALE_TXT)
V_center = np.array([cx, cy, cz])  # 模型的几何中心（中心化的偏移量）

print(f"obj_scale: {obj_scale}")
print(f"模型几何中心 V_center: {V_center}")

# 2. 模型中心化（和init_SDFR逻辑完全一致）
model_pts = np.asarray(pcd_model_raw.points)
model_pts_centered = model_pts - V_center  # 核心：中心化操作

# 3. 🔥 核心修复：修正GT位姿的平移量！
gt_R = gt_pose[:3, :3]
gt_t_raw = gt_pose[:3, 3]
# 公式推导：
# 原始模型：P_scene = gt_R @ P_raw + gt_t_raw
# 中心化后：P_raw = P_centered + V_center
# 代入得：P_scene = gt_R @ (P_centered + V_center) + gt_t_raw
#        = gt_R @ P_centered + (gt_R @ V_center + gt_t_raw)
# 所以修正后的平移量是：gt_t_corrected = gt_R @ V_center + gt_t_raw
gt_t_corrected = gt_R @ V_center + gt_t_raw

print(f"原始GT平移: {gt_t_raw}")
print(f"修正后GT平移: {gt_t_corrected}")

# 4. 用修正后的GT位姿变换模型
model_pts_gt_scene = (gt_R @ model_pts_centered.T).T + gt_t_corrected

# 5. 生成三个点云
pcd_target = o3d.geometry.PointCloud(pcd_target_scene)
pcd_target.paint_uniform_color([0, 1, 0])  # 绿色：场景目标点云

pcd_gt = o3d.geometry.PointCloud()
pcd_gt.points = o3d.utility.Vector3dVector(model_pts_gt_scene)
pcd_gt.paint_uniform_color([0, 0, 1])  # 蓝色：修正后的GT位姿模型

pcd_init = o3d.geometry.PointCloud()
pcd_init.points = o3d.utility.Vector3dVector(model_pts_centered)
pcd_init.paint_uniform_color([1, 0, 0])  # 红色：初始中心化模型

# 打印三个点云的中心，验证平移
print(f"\n📊 点云中心坐标：")
print(f"绿色目标点云中心: {np.mean(np.asarray(pcd_target.points), axis=0)}")
print(f"蓝色GT点云中心: {np.mean(np.asarray(pcd_gt.points), axis=0)}")
print(f"红色初始模型中心: {np.mean(np.asarray(pcd_init.points), axis=0)}")

print("\n✅ 现在绿色和蓝色应该完美重合了！")
print("绿色 = 相机观测目标")
print("蓝色 = 修正后GT位姿（标准答案）")
print("红色 = 初始模型位置")

o3d.visualization.draw_geometries([pcd_target, pcd_gt, pcd_init], window_name="最终修复版诊断")
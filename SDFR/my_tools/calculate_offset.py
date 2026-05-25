# 这段代码会加载你的物体模型，算出 **“物体实际中心” 相对于 “当前模型坐标系原点” 的固定偏移 Δ**，并保存下来

import open3d as o3d
import numpy as np

# ================= 配置区域 =================
obj_path = "/home/qin-desktop-5060/workspace/SDFR/datasets/models/lm/obj_000001.obj"
scale_txt_path = obj_path.replace(".obj", "_scale.txt")
save_offset_path = "/home/qin-desktop-5060/workspace/SDFR/datasets/obj_000001_offset.npy"
# ===========================================

# 1. 加载模型和缩放信息（和你推理代码保持一致）
obj_mesh = o3d.io.read_triangle_mesh(obj_path)
scale_info = np.loadtxt(scale_txt_path)
V_center = scale_info[1:4]

# 2. 做同样的中心化处理（和你推理代码完全对齐）obj单位是mm时，要除1000，是m时，不用除1000
# V = (np.array(obj_mesh.vertices) - V_center) / 1000.0
V = (np.array(obj_mesh.vertices) - V_center)
obj_mesh.vertices = o3d.utility.Vector3dVector(V)

# 3. 计算【物体 AABB 包围盒中心】在当前物体坐标系下的坐标 (这就是 Δ)
pts = np.asarray(obj_mesh.vertices)
min_xyz = np.min(pts, axis=0)
max_xyz = np.max(pts, axis=0)
center_obj = (min_xyz + max_xyz) / 2.0

# 这个 delta 就是：物体中心 相对于 模型原点 的偏移
delta = center_obj

print("="*60)
print("✅ 偏移量计算完成")
print("="*60)
print(f"📦 物体包围盒最小点: {min_xyz}")
print(f"📦 物体包围盒最大点: {max_xyz}")
print(f"\n📍 偏移量 Δ (物体中心在物体坐标系下的坐标):")
print(delta)
print(f"\n💾 已保存至: {save_offset_path}")
print("="*60)

# 4. 可视化验证（红点=模型原点，绿点=物体中心）
origin_pcd = o3d.geometry.PointCloud()
origin_pcd.points = o3d.utility.Vector3dVector([[0,0,0]])
origin_pcd.paint_uniform_color([1,0,0])   # 红色：当前模型原点

center_pcd = o3d.geometry.PointCloud()
center_pcd.points = o3d.utility.Vector3dVector([center_obj])
center_pcd.paint_uniform_color([0,1,0])   # 绿色：你想要的物体中心

# 画个小坐标系
coord = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05)

o3d.visualization.draw_geometries([obj_mesh, origin_pcd, center_pcd, coord], window_name="偏移量验证 (红=原点, 绿=中心)")

# 5. 保存偏移量
np.save(save_offset_path, delta)
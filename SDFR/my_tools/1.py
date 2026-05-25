# 查看初始位姿与真实位姿的误差
import open3d as o3d
import numpy as np

# ==============================================
# 1. 配置参数（仅需修改这里的OBJ文件路径）
# ==============================================
# 你的OBJ模型文件路径（替换成你瓶子/齿轮的obj路径）
obj_file_path = "/home/qin-desktop-5060/workspace/SDFR/datasets/models/lm/obj_000001.obj"

# 是否和你的SDFR代码保持完全一致的尺度归一化（必须开！）
align_with_sdfr_scale = True

# ==============================================
# 2. 你提供的GT位姿和初始位姿
# ==============================================
gt_pose = np.array([
    [-9.928588867187500000e-01, -1.156553328037261963e-01, 2.924402058124542236e-02, 0.000000000000000000e+00],
    [-1.117422506213188171e-01, 9.874614477157592773e-01, 1.115063428878784180e-01, 0.000000000000000000e+00],
    [-4.177364334464073181e-02, 1.074422597885131836e-01, -9.933333396911621094e-01, 5.000000000000000000e-01],
    [0.000000000000000000e+00, 0.000000000000000000e+00, 0.000000000000000000e+00, 1.000000000000000000e+00]
], dtype=np.float64)

init_pose = np.array([
    [-9.824457564021054079e-01, -1.727637725604839192e-01, -7.037863032048294332e-02, 1.860200221331578460e-02],
    [-1.826874917845205670e-01, 9.673768983122678078e-01, 1.755201467295135487e-01, 2.734276041666319160e-03],
    [3.775914275475542037e-02, 1.852963070914037669e-01, -9.819569886060417474e-01, 5.181836631523681902e-01],
    [0.000000000000000000e+00, 0.000000000000000000e+00, 0.000000000000000000e+00, 1.000000000000000000e+00]
], dtype=np.float64)

# ==============================================
# 3. 加载模型 + 对齐SDFR尺度
# ==============================================
mesh_original = o3d.io.read_triangle_mesh(obj_file_path)
mesh_original.compute_vertex_normals()

if align_with_sdfr_scale:
    mesh_original.scale(0.001, center=np.zeros(3))
    V = np.asarray(mesh_original.vertices)
    V_max, V_min = V.max(axis=0), V.min(axis=0)
    V_center = (V_max + V_min) / 2.0
    mesh_original.translate(-V_center)
    max_dist = np.max(np.linalg.norm(V - V_center, axis=1))
    obj_scale = 1.0 / max_dist
    mesh_original.scale(obj_scale, center=np.zeros(3))
    print(f"已对齐SDFR尺度 | 缩放系数: {obj_scale:.4f}")

# 复制模型
mesh_gt = o3d.geometry.TriangleMesh(mesh_original)
mesh_init = o3d.geometry.TriangleMesh(mesh_original)

# 应用位姿
mesh_gt.transform(gt_pose)
mesh_init.transform(init_pose)

# 颜色：绿色=GT，红色=Init
mesh_gt.paint_uniform_color([0, 1, 0])
mesh_init.paint_uniform_color([1, 0, 0])

# ==============================================
# 4. 坐标系
# ==============================================
coord_world = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0,0,0])
coord_gt = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.08)
coord_gt.transform(gt_pose)
coord_init = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.08)
coord_init.transform(init_pose)

# ==============================================
# 5. 可视化
# ==============================================
print("="*50)
print("绿色 = GT 真值位姿")
print("红色 = Init 初始位姿")
print("旋转误差 ≈ 5.7° | 平移误差 ≈ 2.6 cm")
print("="*50)

o3d.visualization.draw_geometries(
    [mesh_gt, mesh_init, coord_world, coord_gt, coord_init],
    window_name="GT vs Init 位姿对比",
    width=1280, height=720, mesh_show_back_face=True
)
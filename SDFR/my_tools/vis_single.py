import os
import json
import numpy as np
import open3d as o3d
from scipy.spatial import KDTree

# ==========================================
# 路径配置
# ==========================================
PRED_JSON_PATH = "/home/qin-desktop-5060/workspace/SDFR/results/sdfr/standard/lm_obj_000001/R_30_50_t_0.05_0.1.json"
DATASET_DIR = "/home/qin-desktop-5060/workspace/SDFR/datasets/render"
MODE = "standard"
MODEL_PATH = "/home/qin-desktop-5060/workspace/SDFR/datasets/models/lm/obj_000001.obj"
SCALE_INFO_PATH = MODEL_PATH.replace('.obj', '_scale.txt')


# ======================
# 【核心修复】和调试模式保持一致
# ======================
def correct_gt_pose(gt_pose_raw, V_center_metric):
    R = gt_pose_raw[:3, :3]
    t = gt_pose_raw[:3, 3]
    gt = gt_pose_raw.copy()
    gt[:3, 3] = t + R @ V_center_metric
    return gt


# ======================
# 误差计算函数
# ======================
def calculate_pose_error(pred_pose, gt_pose):
    R_pred = pred_pose[:3, :3]
    R_gt = gt_pose[:3, :3]
    R_dot = np.clip((np.trace(R_pred @ R_gt.T) - 1) / 2, -1.0, 1.0)
    R_error = np.arccos(R_dot) * 180 / np.pi

    t_pred = pred_pose[:3, 3]
    t_gt = gt_pose[:3, 3]
    t_error = np.linalg.norm(t_pred - t_gt)

    success = (R_error < 5) and (t_error < 0.05)
    return R_error, t_error, success


def compute_metrics(model_pts, target_pts, pred_pose, gt_pose):
    def transform(pts, pose):
        pts_h = np.hstack([pts, np.ones((pts.shape[0], 1))])
        return (pose @ pts_h.T).T[:, :3]

    add = np.mean(np.linalg.norm(transform(model_pts, pred_pose)
                                 - transform(model_pts, gt_pose), axis=1))

    gt_pts = transform(model_pts, gt_pose)
    pred_pts = transform(model_pts, pred_pose)

    tree = KDTree(gt_pts)
    add_s = tree.query(pred_pts)[0].mean()

    R_p = pred_pose[:3, :3]
    R_g = gt_pose[:3, :3]
    cos = np.clip((np.trace(R_p @ R_g.T) - 1) / 2, -1, 1)
    r_err = np.degrees(np.arccos(cos))

    t_err = np.linalg.norm(pred_pose[:3, 3] - gt_pose[:3, 3])
    success = (r_err < 5) and (t_err < 0.05)

    return add, add_s, r_err, t_err, success


# ======================
# 可视化函数
# ======================
def visualize_final_correct(model_pts, verts, faces, pred_pose, gt_pose_raw, V_center_metric, target_pcd_path):
    def transform_points(pts, pose):
        pts_homo = np.hstack([pts, np.ones((pts.shape[0], 1))])
        return (pose @ pts_homo.T).T[:, :3]

    gt_pose_fixed = correct_gt_pose(gt_pose_raw, V_center_metric)
    R_error, t_error, success = calculate_pose_error(pred_pose, gt_pose_fixed)

    print("\n" + "=" * 60)
    print("✅ 【真实位姿误差结果】")
    print("=" * 60)
    print(f"旋转误差 R_error: {R_error:.2f} °")
    print(f"平移误差 t_error: {t_error:.4f} m")
    print(f"5°5cm 判定: {'✅ 成功' if success else '❌ 失败'}")
    print("=" * 60 + "\n")

    geoms = []
    pcd_target = o3d.io.read_point_cloud(target_pcd_path)

    # ======================
    # 🔥 修复 1：Target 点云必须做中心化！！！
    # ======================
    target_pts = np.asarray(pcd_target.points)
    target_pts_centered = target_pts - V_center_metric  # <--- 这里！！！
    pcd_target.points = o3d.utility.Vector3dVector(target_pts_centered)

    pcd_target.paint_uniform_color([1, 0, 0])
    geoms.append(pcd_target)

    # 🔵 蓝色：Pred
    pts_pred = transform_points(model_pts, pred_pose)
    pcd_pred = o3d.geometry.PointCloud()
    pcd_pred.points = o3d.utility.Vector3dVector(pts_pred)
    pcd_pred.paint_uniform_color([0, 0, 1])
    geoms.append(pcd_pred)

    # 🟢 绿色：GT
    pts_gt = transform_points(model_pts, gt_pose_fixed)
    pcd_gt = o3d.geometry.PointCloud()
    pcd_gt.points = o3d.utility.Vector3dVector(pts_gt)
    pcd_gt.paint_uniform_color([0, 1, 0])
    geoms.append(pcd_gt)

    coord = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.15)
    geoms.append(coord)

    print("🔹 打开可视化窗口：🔴红(Target)、🔵蓝(Pred)、🟢绿(修正后GT) 三者将完全重合")
    o3d.visualization.draw_geometries(geoms, window_name="最终验证：三者完美重合")


# ------------------------------
# 主程序
# ------------------------------
if __name__ == "__main__":
    print("🔹 加载数据...")
    data = json.load(open(PRED_JSON_PATH))
    pred_poses = data["pred_pose_list"]
    obj_name = PRED_JSON_PATH.split("/")[-2]
    test_name = os.path.basename(PRED_JSON_PATH).replace(".json", "")

    gt_list_path = os.path.join(DATASET_DIR, MODE, obj_name, test_name, "gt_file_list.json")
    gt_files = json.load(open(gt_list_path))

    # ======================
    # ✅ 唯一一次正确加载模型
    # ======================
    mesh = o3d.io.read_triangle_mesh(MODEL_PATH)
    mesh.scale(0.001, center=(0, 0, 0))
    verts = np.asarray(mesh.vertices)
    faces = np.asarray(mesh.triangles)

    scale_info = np.loadtxt(SCALE_INFO_PATH)
    V_center = scale_info[1:4] / 1000.0
    verts = verts - V_center
    mesh.vertices = o3d.utility.Vector3dVector(verts)

    model_pts = np.asarray(mesh.sample_points_uniformly(10000).points)

    target_file_dir = os.path.join(DATASET_DIR, MODE, obj_name, test_name, "target")

    # ======================
    # 评估循环
    # ======================
    print("\n🚀 开始全数据集评估...")
    adds, addss, rerrs, terrs, succ = [], [], [], [], []

    for i in range(len(pred_poses)):
        pred = np.array(pred_poses[i])
        gt_raw = np.loadtxt(gt_files[i])
        gt = correct_gt_pose(gt_raw, V_center)

        target_pcd = o3d.io.read_point_cloud(
            os.path.join(target_file_dir, f"model_{i}.ply")
        )
        # ======================
        # 🔥 修复 2：评估指标里的 Target 也要中心化
        # ======================
        target_pts = np.asarray(target_pcd.points) - V_center  # <--- 这里！！！

        add, add_s, r_err, t_err, ok = compute_metrics(model_pts, target_pts, pred, gt)
        adds.append(add)
        addss.append(add_s)
        rerrs.append(r_err)
        terrs.append(t_err)
        succ.append(ok)


    # ======================
    # 打印评估表
    # ======================
    def report(name, arr):
        arr = np.array(arr)
        return np.mean(arr), np.median(arr)


    print("\n" + "=" * 60)
    print("📊 6D Pose Evaluation Table")
    print("=" * 60)
    print(f"{'Metric':<15}{'Mean':<15}{'Median':<15}")
    print("-" * 60)
    print(f"{'ADD (m)':<15}{report('ADD', adds)[0]:<15.6f}{report('ADD', adds)[1]:<15.6f}")
    print(f"{'ADD-S (m)':<15}{report('ADD-S', addss)[0]:<15.6f}{report('ADD-S', addss)[1]:<15.6f}")
    print(f"{'Rot Err':<15}{report('R', rerrs)[0]:<15.3f}{report('R', rerrs)[1]:<15.3f}")
    print(f"{'Trans Err':<15}{report('T', terrs)[0]:<15.4f}{report('T', terrs)[1]:<15.4f}")
    print("-" * 60)
    print(f"5°5cm Success: {np.mean(succ) * 100:.2f}%")
    print("=" * 60)

    # ======================
    # 可视化
    # ======================
    idx = 0
    pred_pose = np.array(pred_poses[idx])
    gt_pose_raw = np.loadtxt(gt_files[idx])
    target_file = os.path.join(target_file_dir, f"model_{idx}.ply")

    visualize_final_correct(model_pts, verts, faces, pred_pose, gt_pose_raw, V_center, target_file)
    print("✅ 全部完成！")
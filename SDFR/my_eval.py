import os
import json
import numpy as np
import torch
from scipy import spatial
import open3d as o3d
import glob

# ====================== 【只用改这里！】 ======================
# 1. 你之前推理保存的 JSON（如果没有，用我下面给的生成函数）
PREDICTED_JSON = "/home/qin-desktop-5060/workspace/SDFR/results/sdfr/standard/lm_obj_000001/R_5_30_t_0.01_0.05.json"

# 2. 你的 GT 位姿文件列表 - 自动扫描目录下的所有txt文件
GT_POSE_DIR = "/home/qin-desktop-5060/workspace/SDFR/datasets/render/standard/lm_obj_000001/R_5_30_t_0.01_0.05/gt_pose/"

# 自动获取目录下所有txt文件并排序
GT_FILE_LIST_TXT = sorted(glob.glob(os.path.join(GT_POSE_DIR, "*.txt")))

# 如果列表为空，提示错误
if not GT_FILE_LIST_TXT:
    raise FileNotFoundError(f"❌ 在目录 {GT_POSE_DIR} 中未找到任何 .txt 文件！")

# 3. 你的模型文件
OBJ_FILE = "/home/qin-desktop-5060/workspace/SDFR/datasets/models/lm/obj_000001.obj"
MODELS_INFO_JSON = "/home/qin-desktop-5060/workspace/SDFR/datasets/models/lm/models_info.json"


def compute_metrics(pred_pose, gt_pose, model_pts, diameter):
    """ 计算 ADD / ADD-S / 5cm5deg """
    # 5cm 5deg
    R_pred, t_pred = pred_pose[:3, :3], pred_pose[:3, 3]
    R_gt, t_gt = gt_pose[:3, :3], gt_pose[:3, 3]

    eps = 1e-6
    r_loss = (np.trace(R_pred @ R_gt.T) - 1) / 2
    r_loss = np.clip(r_loss, -1 + eps, 1 - eps)
    R_err = np.arccos(r_loss) * 180 / np.pi
    t_err = np.linalg.norm(t_pred - t_gt)
    is_5cm5deg = (R_err < 5) and (t_err < 0.05)

    # ADD
    pts_pred = (R_pred @ model_pts.T + t_pred[:, None]).T
    pts_gt = (R_gt @ model_pts.T + t_gt[:, None]).T
    add = np.mean(np.linalg.norm(pts_pred - pts_gt, axis=1))
    is_add = add < 0.1 * diameter

    # ADD-S
    tree = spatial.cKDTree(pts_pred)
    dists, _ = tree.query(pts_gt, k=1)
    adds = np.mean(dists)
    is_adds = adds < 0.1 * diameter

    return is_5cm5deg, is_add, is_adds, R_err, t_err, add, adds


if __name__ == '__main__':
    print("=" * 60)
    print("✅ 开始评估你自己的瓶子模型...")
    print("=" * 60)

    # 加载模型
    mesh = o3d.io.read_triangle_mesh(OBJ_FILE)
    model_pts = np.array(mesh.sample_points_uniformly(10000).points)
    models_info = json.load(open(MODELS_INFO_JSON))
    diameter = models_info["1"]["diameter"] / 1000.0  # mm -> m

    # 加载推理结果
    if os.path.exists(PREDICTED_JSON):
        data = json.load(open(PREDICTED_JSON))
        pred_poses = data["pred_pose_list"]
    else:
        print("❌ 没有找到推理结果 JSON！")
        print("请先运行推理脚本！")
        exit()

    # 打印GT文件数量信息
    print(f"📁 找到 {len(GT_FILE_LIST_TXT)} 个GT位姿文件")
    print(f"📊 预测姿态数量: {len(pred_poses)}")

    # 评估
    count_5cm5deg = 0
    count_add = 0
    count_adds = 0
    n = len(pred_poses)

    for i in range(n):
        pred_pose = np.array(pred_poses[i])
        gt_file = GT_FILE_LIST_TXT[i % len(GT_FILE_LIST_TXT)]

        # 检查文件是否存在
        if not os.path.isfile(gt_file):
            print(f"⚠️  警告: GT文件不存在，跳过: {gt_file}")
            continue

        gt_pose = np.loadtxt(gt_file)

        is_5cm5deg, is_add, is_adds, R_err, t_err, add, adds = compute_metrics(
            pred_pose, gt_pose, model_pts, diameter
        )

        if is_5cm5deg: count_5cm5deg += 1
        if is_add: count_add += 1
        if is_adds: count_adds += 1

        print(
            f"  [{i + 1}/{n}] R_err: {R_err:5.2f}° | t_err: {t_err * 100:5.2f}cm | ADD: {add * 1000:6.2f}mm | ADD-S: {adds * 1000:6.2f}mm")

    # 输出最终结果
    print("\n" + "=" * 60)
    print("✅ 评估完成！最终结果：")
    print(f"  5cm 5deg: {count_5cm5deg / n:.4f}")
    print(f"  ADD:      {count_add / n:.4f}")
    print(f"  ADD-S:    {count_adds / n:.4f}")
    print("=" * 60)

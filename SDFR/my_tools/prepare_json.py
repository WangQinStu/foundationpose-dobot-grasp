import open3d as o3d
import json
import os
import numpy as np
from scipy.spatial import ConvexHull


def calc_model_info(model_path):
    # 读取模型
    mesh = o3d.io.read_triangle_mesh(model_path)
    vertices = np.asarray(mesh.vertices).copy()
    n_vertices = len(vertices)

    print(f"  顶点数: {n_vertices}")

    min_xyz = np.min(vertices, axis=0)
    max_xyz = np.max(vertices, axis=0)
    size_xyz = max_xyz - min_xyz

    # 计算直径
    if n_vertices > 10000:
        hull = ConvexHull(vertices)
        hull_vertices = vertices[hull.vertices]
        if len(hull_vertices) > 5000:
            indices = np.random.choice(len(hull_vertices), 5000, replace=False)
            hull_vertices = hull_vertices[indices]
        dists = np.linalg.norm(hull_vertices[:, None] - hull_vertices, axis=2)
        diameter = np.max(dists)
    else:
        dists = np.linalg.norm(vertices[:, None] - vertices, axis=2)
        diameter = np.max(dists)

    return {
        "diameter": float(diameter),
        "min_x": float(min_xyz[0]),
        "min_y": float(min_xyz[1]),
        "min_z": float(min_xyz[2]),
        "size_x": float(size_xyz[0]),
        "size_y": float(size_xyz[1]),
        "size_z": float(size_xyz[2])
    }


if __name__ == "__main__":
    models_dir = "/home/qin-desktop-5060/workspace/SDFR/datasets/models/lm"
    output_json = "/home/qin-desktop-5060/workspace/SDFR/datasets/models/lm/models_info.json"

    models_info = {}
    obj_id = 1

    for file_name in sorted(os.listdir(models_dir)):
        if file_name.endswith((".ply", ".obj")):
            model_full_path = os.path.join(models_dir, file_name)

            print(f"处理: {file_name}")
            try:
                model_info = calc_model_info(model_full_path)
                models_info[str(obj_id)] = model_info
                print(f"  ✓ ID {obj_id} 完成 | 直径: {model_info['diameter']:.2f}")
                obj_id += 1
            except Exception as e:
                print(f"  ✗ 失败: {e}")

    with open(output_json, "w", encoding="utf-8") as f:
        json.dump(models_info, f, indent=4, ensure_ascii=False)

    print(f"\n✅ 完成！生成 JSON: {output_json}")
    print(f"✅ 共处理物体数量: {len(models_info)}")
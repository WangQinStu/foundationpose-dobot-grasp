# obj模型可视化

import os
import sys
import numpy as np
import open3d as o3d
from pathlib import Path


def visualize_obj(obj_path):
    """
    使用Open3D可视化OBJ文件

    Args:
        obj_path: OBJ文件路径
    """
    print(f"📂 加载OBJ文件: {obj_path}")

    if not os.path.exists(obj_path):
        print(f"❌ 文件不存在: {obj_path}")
        return

    # 读取OBJ网格
    mesh = o3d.io.read_triangle_mesh(obj_path)

    if mesh.is_empty():
        print("❌ 无法读取网格或网格为空")
        return

    print(f"✅ 成功加载网格")
    print(f"   - 顶点数: {len(mesh.vertices)}")
    print(f"   - 三角形数: {len(mesh.triangles)}")

    # 计算并显示边界框信息
    bbox = mesh.get_axis_aligned_bounding_box()
    min_bound = bbox.get_min_bound()
    max_bound = bbox.get_max_bound()
    print(f"   - 包围盒范围:")
    print(f"     X: [{min_bound[0]:.4f}, {max_bound[0]:.4f}]")
    print(f"     Y: [{min_bound[1]:.4f}, {max_bound[1]:.4f}]")
    print(f"     Z: [{min_bound[2]:.4f}, {max_bound[2]:.4f}]")

    # 设置颜色(如果没有顶点颜色)
    if not mesh.has_vertex_colors():
        mesh.paint_uniform_color([0.7, 0.7, 0.7])  # 灰色

    # 计算法向量以获得更好的渲染效果
    mesh.compute_vertex_normals()

    # 创建坐标轴
    coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)

    # 创建窗口标题
    window_name = f"OBJ Viewer - {os.path.basename(obj_path)}"

    print(f"\n🖼️  打开3D可视化窗口...")
    print(f"   - 鼠标左键: 旋转")
    print(f"   - 鼠标右键: 平移")
    print(f"   - 滚轮: 缩放")
    print(f"   - 按 'W': 切换线框模式")
    print(f"   - 按 'N': 显示/隐藏法向量")
    print(f"   - 按 'H': 显示帮助信息")
    print(f"   - 按 'Q' 或关闭窗口: 退出\n")

    # 显示网格
    o3d.visualization.draw_geometries(
        [mesh, coord_frame],
        window_name=window_name,
        width=1024,
        height=768
    )


def visualize_multiple_objs(obj_paths):
    """
    同时可视化多个OBJ文件(不同颜色)

    Args:
        obj_paths: OBJ文件路径列表
    """
    geometries = []
    colors = [
        [1.0, 0.0, 0.0],  # 红色
        [0.0, 1.0, 0.0],  # 绿色
        [0.0, 0.0, 1.0],  # 蓝色
        [1.0, 1.0, 0.0],  # 黄色
        [1.0, 0.0, 1.0],  # 品红
        [0.0, 1.0, 1.0],  # 青色
    ]

    for i, obj_path in enumerate(obj_paths):
        print(f"📂 加载: {obj_path}")
        mesh = o3d.io.read_triangle_mesh(obj_path)

        if mesh.is_empty():
            print(f"   ⚠️  跳过空网格")
            continue

        # 设置不同颜色
        color = colors[i % len(colors)]
        mesh.paint_uniform_color(color)
        mesh.compute_vertex_normals()

        geometries.append(mesh)
        print(f"   ✅ 顶点:{len(mesh.vertices)}, 面:{len(mesh.triangles)}")

    if not geometries:
        print("❌ 没有成功加载任何网格")
        return

    # 添加坐标轴
    coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
    geometries.append(coord_frame)

    window_name = "Multi-OBJ Viewer"
    print(f"\n🖼️  打开多模型可视化窗口 (共{len(geometries) - 1}个模型)\n")

    o3d.visualization.draw_geometries(
        geometries,
        window_name=window_name,
        width=1280,
        height=960
    )


if __name__ == "__main__":
    # 默认OBJ路径
    default_obj = "/home/qin-desktop-5060/workspace/SDFR/datasets/models/lm/obj_000001.obj"

    if len(sys.argv) > 1:
        # 命令行参数指定路径
        obj_files = sys.argv[1:]
        print(f"🔍 查看 {len(obj_files)} 个OBJ文件\n")

        if len(obj_files) == 1:
            visualize_obj(obj_files[0])
        else:
            visualize_multiple_objs(obj_files)
    else:
        # 使用默认路径
        if os.path.exists(default_obj):
            print("🔍 使用默认OBJ文件\n")
            visualize_obj(default_obj)
        else:
            print(f"⚠️  默认文件不存在: {default_obj}")
            print("\n用法:")
            print(f"  python view_obj.py                          # 查看默认OBJ")
            print(f"  python view_obj.py path/to/model.obj        # 查看单个OBJ")
            print(f"  python view_obj.py model1.obj model2.obj    # 查看多个OBJ")

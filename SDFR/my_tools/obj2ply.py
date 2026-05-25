import open3d as o3d

def obj2ply(obj_path, ply_path):
    # 读取 OBJ
    mesh = o3d.io.read_triangle_mesh(obj_path)
    # 保存为 PLY
    o3d.io.write_triangle_mesh(ply_path, mesh)
    print(f"转换完成: {obj_path} -> {ply_path}")

# 使用示例
obj2ply("/home/qin-desktop-5060/workspace/SDFR/datasets/models/lm/obj_000001.obj", "/home/qin-desktop-5060/workspace/SDFR/datasets/models/lm/obj_000001.ply")
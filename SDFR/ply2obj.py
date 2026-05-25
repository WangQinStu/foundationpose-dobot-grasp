import os
import numpy as np
from glob import glob
from plyfile import PlyData
from tqdm import tqdm

def write_obj(verts, faces, obj_path):
    """
    Write .obj file
    """
    assert obj_path[-4:] == '.obj'
    with open(obj_path, 'w') as fp:
        for v in verts:
            fp.write('v %f %f %f\n' % (v[0], v[1], v[2]))
        if faces is not None and len(faces) > 0:
            for f in faces + 1:
                fp.write('f %d %d %d\n' % (f[0], f[1], f[2]))

def ply2obj(ply_name, obj_name):
    plydata = PlyData.read(ply_name)
    pc = plydata['vertex'].data

    # ===================== 【修复在这里】 =====================
    # 自动取前3列 xyz，不管后面有多少列（永远不报错）
    # ==========================================================
    pc_array = np.array([[x[0], x[1], x[2]] for x in pc])

    # 安全读取 face（有些 PLY 是点云，没有 face）
    face_array = None
    if 'face' in plydata:
        faces = plydata['face'].data
        face_array = np.array([face[0] for face in faces], dtype=np.int64)

    write_obj(pc_array, face_array, obj_name)

import argparse
import glob

if __name__ == "__main__":
    # 这里换成你自己的 PLY 路径即可
    filelist = ['/home/qin-desktop-5060/workspace/SDFR/datasets/models/lm/obj_000001.ply']

    for i in tqdm(range(len(filelist))):
        if os.path.exists(filelist[i].replace('.ply', '.obj')):
            continue
        ply2obj(filelist[i], filelist[i].replace('.ply', '.obj'))
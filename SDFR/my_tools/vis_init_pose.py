import open3d as o3d
import open3d.visualization.gui as gui
import open3d.visualization.rendering as rendering
import numpy as np

# ===================== 【你只改这里】 =====================
ply_path = "/home/qin-desktop-5060/workspace/SDFR/my_tools/real_data/000010.ply"
obj_path = "/home/qin-desktop-5060/workspace/SDFR/datasets/models/lm/obj_000002.obj"
scale_file = obj_path.replace(".obj", "_scale.txt")
# ==========================================================

pcd = o3d.io.read_point_cloud(ply_path)
mesh = o3d.io.read_triangle_mesh(obj_path)
scale_info = np.loadtxt(scale_file)
V_center = scale_info[1:4]

verts = np.array(mesh.vertices)

# 单位mm时，除1000；单位是m时，不除1000
# verts_centered = (verts - V_center) / 1000.0
verts_centered = (verts - V_center)
mesh_pcd = o3d.geometry.PointCloud()
mesh_pcd.points = o3d.utility.Vector3dVector(verts_centered)

def euler_to_matrix(roll, pitch, yaw):
    R_x = np.array([[1, 0, 0],
                    [0, np.cos(roll), -np.sin(roll)],
                    [0, np.sin(roll), np.cos(roll)]])
    R_y = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                    [0, 1, 0],
                    [-np.sin(pitch), 0, np.cos(pitch)]])
    R_z = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                    [np.sin(yaw), np.cos(yaw), 0],
                    [0, 0, 1]])
    return R_z @ R_y @ R_x

# ---------------------
# GUI 位姿调整器
# ---------------------
class PoseAdjusterApp:
    def __init__(self, template_pcd, obs_pcd):
        self.template_raw = np.asarray(template_pcd.points)
        self.pcd_obs = obs_pcd

        self.params = {
            'roll': 0.0,
            'pitch': 0.0,
            'yaw': 0.0,
            'tx': 0.0,
            'ty': 0.0,
            'tz': 0.3,
            'scale': 1.0
        }

        self.app = gui.Application.instance
        self.app.initialize()
        self.window = self.app.create_window("Pose Adjuster", 1400, 800)
        self._setup_scene()
        self._setup_panel()
        self._update_model()

    def _setup_scene(self):
        self.widget3d = gui.SceneWidget()
        self.window.add_child(self.widget3d)
        self.scene = rendering.Open3DScene(self.window.renderer)
        self.widget3d.scene = self.scene
        self.scene.set_background([1,1,1,1])

        bbox = self.pcd_obs.get_axis_aligned_bounding_box()
        self.widget3d.setup_camera(60, bbox, bbox.get_center())

        # 观测点云（红色）
        mat_obs = rendering.MaterialRecord()
        mat_obs.shader = "defaultUnlit"
        mat_obs.point_size = 3
        self.pcd_obs.paint_uniform_color([1,0,0])
        self.scene.add_geometry("obs", self.pcd_obs, mat_obs)

        # 模型点云（青色）
        self.pcd_tpl = o3d.geometry.PointCloud()
        mat_tpl = rendering.MaterialRecord()
        mat_tpl.shader = "defaultUnlit"
        mat_tpl.base_color = [0,1,1,1]
        mat_tpl.point_size = 3
        self.mat_tpl = mat_tpl

        coord = o3d.geometry.TriangleMesh.create_coordinate_frame(0.15)
        self.scene.add_geometry("coord", coord, rendering.MaterialRecord())

    def _setup_panel(self):
        panel = gui.Vert(0, gui.Margins(10,10,10,10))
        panel.add_child(gui.Label("Rotation (deg)"))

        self.sliders = {}
        for name, vmin, vmax in [
            ("roll",-180,180),
            ("pitch",-180,180),
            ("yaw",-180,180)
        ]:
            row = gui.Horiz(5)
            row.add_child(gui.Label(f"{name}:"))
            s = gui.Slider(gui.Slider.DOUBLE)
            s.set_limits(vmin, vmax)
            s.double_value = self.params[name]
            s.set_on_value_changed(self._mk_cb(name))
            row.add_child(s)
            panel.add_child(row)
            self.sliders[name] = s

        panel.add_child(gui.Label("Translation"))
        for name, vmin, vmax in [
            ("tx",-1,1),
            ("ty",-1,1),
            ("tz",0,1)
        ]:
            row = gui.Horiz(5)
            row.add_child(gui.Label(f"{name}:"))
            s = gui.Slider(gui.Slider.DOUBLE)
            s.set_limits(vmin, vmax)
            s.double_value = self.params[name]
            s.set_on_value_changed(self._mk_cb(name))
            row.add_child(s)
            panel.add_child(row)
            self.sliders[name] = s

        panel.add_child(gui.Label("Scale"))
        row = gui.Horiz(5)
        row.add_child(gui.Label("scale:"))
        s = gui.Slider(gui.Slider.DOUBLE)
        s.set_limits(0.1, 2.0)
        s.double_value = self.params["scale"]
        s.set_on_value_changed(self._mk_cb("scale"))
        row.add_child(s)
        panel.add_child(row)
        self.sliders["scale"] = s

        self.window.set_on_layout(self._on_layout)
        self.window.add_child(panel)
        self.panel = panel

    def _mk_cb(self, name):
        def cb(v):
            self.params[name] = v
            self._update_model()
        return cb

    def _on_layout(self, ctx):
        r = self.window.content_rect
        self.widget3d.frame = r
        self.panel.frame = gui.Rect(r.width-280, r.y, 280, r.height)

    def _update_model(self):
        r = np.radians(self.params['roll'])
        p = np.radians(self.params['pitch'])
        y = np.radians(self.params['yaw'])
        t = np.array([self.params['tx'], self.params['ty'], self.params['tz']])
        s = self.params['scale']

        R = euler_to_matrix(r,p,y)
        pts = s * (self.template_raw @ R.T) + t

        self.pcd_tpl.points = o3d.utility.Vector3dVector(pts)
        self.scene.remove_geometry("model")
        self.scene.add_geometry("model", self.pcd_tpl, self.mat_tpl)

    def run(self):
        self.app.run()
        return self.params


app = PoseAdjusterApp(mesh_pcd, pcd)
params = app.run()

# ---------------------
# 生成最终位姿
# ---------------------
r = np.radians(params['roll'])
p = np.radians(params['pitch'])
y = np.radians(params['yaw'])
R = euler_to_matrix(r,p,y)
t = np.array([params['tx'], params['ty'], params['tz']])
s = params['scale']

initial_pose = np.eye(4)
initial_pose[:3,:3] = R
initial_pose[:3,3] = t

print("\n✅ 你的 initial_pose 已生成！")
print(initial_pose)
np.savetxt("initial_pose.txt", initial_pose)
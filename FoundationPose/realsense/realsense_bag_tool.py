import os, cv2, time, argparse
import numpy as np
import pyrealsense2 as rs


def ensure_dir(p): os.makedirs(p, exist_ok=True)


def save_K(intr, path):
    K = np.array([[intr.fx, 0, intr.ppx],
                  [0, intr.fy, intr.ppy],
                  [0, 0, 1]])
    np.savetxt(path, K, fmt="%.18e")


# ===================== 录制 =====================
def record_bag(path, w=848, h=480, fps=30, duration=10):
    ensure_dir(os.path.dirname(path) or ".")

    pipe, cfg = rs.pipeline(), rs.config()
    cfg.enable_stream(rs.stream.color, w, h, rs.format.bgr8, fps)
    cfg.enable_stream(rs.stream.depth, w, h, rs.format.z16, fps)
    cfg.enable_record_to_file(path)

    prof = pipe.start(cfg)
    scale = prof.get_device().first_depth_sensor().get_depth_scale()
    print(f"[INFO] recording... scale={scale}")

    t0 = time.time()
    while time.time() - t0 < duration:
        frames = pipe.wait_for_frames()
        c, d = frames.get_color_frame(), frames.get_depth_frame()
        if not c or not d: continue

        rgb = np.asanyarray(c.get_data())
        depth = np.asanyarray(d.get_data())
        vis = cv2.applyColorMap(cv2.convertScaleAbs(depth, alpha=0.03), cv2.COLORMAP_JET)

        cv2.imshow("RGB | Depth", np.hstack([rgb, vis]))
        if cv2.waitKey(1) & 0xFF == ord('q'): break

    pipe.stop()
    cv2.destroyAllWindows()
    print("[INFO] done:", path)


# ===================== 提取 =====================
def extract_bag(bag, out, align=True, max_frames=-1):
    if not os.path.isfile(bag): raise FileNotFoundError(bag)

    rgb_dir, d_dir = os.path.join(out, "../demo_data/book_435_work/rgb"), os.path.join(out,
                                                                                       "../demo_data/book_435_work/depth")
    ensure_dir(rgb_dir); ensure_dir(d_dir)

    pipe, cfg = rs.pipeline(), rs.config()
    rs.config.enable_device_from_file(cfg, bag, False)
    prof = pipe.start(cfg)

    prof.get_device().as_playback().set_real_time(False)
    aligner = rs.align(rs.stream.color) if align else None

    intr = prof.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    save_K(intr, os.path.join(out, "cam_K.txt"))

    try:
        scale = prof.get_device().first_depth_sensor().get_depth_scale()
        with open(os.path.join(out, "depth_scale.txt"), "w") as f: f.write(str(scale))
    except: pass

    i = 0
    while True:
        try:
            frames = pipe.wait_for_frames(3000)
        except:
            break

        if aligner: frames = aligner.process(frames)
        c, d = frames.get_color_frame(), frames.get_depth_frame()
        if not c or not d: continue

        cv2.imwrite(f"{rgb_dir}/{i:06d}.png", np.asanyarray(c.get_data()))
        cv2.imwrite(f"{d_dir}/{i:06d}.png", np.asanyarray(d.get_data()))

        if i % 50 == 0: print(f"[{i}]")
        i += 1
        if 0 < max_frames <= i: break

    pipe.stop()
    print(f"[INFO] done: {i} frames")


# ===================== CLI =====================
if __name__ == "__main__":
    p = argparse.ArgumentParser()
    sp = p.add_subparsers(dest="mode", required=True)

    r = sp.add_parser("record")
    r.add_argument("--bag", default="rec.bag")
    r.add_argument("--t", type=int, default=10)

    e = sp.add_parser("extract")
    e.add_argument("--bag", required=True)
    e.add_argument("--out", default="out")
    e.add_argument("--max", type=int, default=-1)

    a = p.parse_args()

    if a.mode == "record":
        record_bag(a.bag, duration=a.t)
    else:
        extract_bag(a.bag, a.out, max_frames=a.max)
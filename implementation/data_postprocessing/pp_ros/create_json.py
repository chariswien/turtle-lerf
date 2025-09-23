#!/usr/bin/env python3

##looks which images are in the image directory and matches the image via the corresponding timestamp to the camerapose at that timestep

import argparse, csv, json, math, os
from bisect import bisect_left

def q_norm(q):
    x,y,z,w = q
    n = math.sqrt(x*x + y*y + z*z + w*w)
    return (0.0,0.0,0.0,1.0) if n==0.0 else (x/n, y/n, z/n, w/n)

def q_to_R(q):
    x,y,z,w = q_norm(q)
    xx,yy,zz = x*x,y*y,z*z
    xy,xz,yz = x*y,x*z,y*z
    wx,wy,wz = w*x,w*y,w*z
    return [
        [1-2*(yy+zz),   2*(xy-wz),     2*(xz+wy)],
        [2*(xy+wz),     1-2*(xx+zz),   2*(yz-wx)],
        [2*(xz-wy),     2*(yz+wx),     1-2*(xx+yy)]
    ]

def R_mul(A, B):
        result = []
        for i in range(3):
            row = []
            for j in range(3):
                val = 0
                for k in range(3):
                    val += A[i][k] * B[k][j]
                row.append(val)
            result.append(row)
        return result

def mat4(R, t):
    return [
        [R[0][0], R[0][1], R[0][2], t[0]],
        [R[1][0], R[1][1], R[1][2], t[1]],
        [R[2][0], R[2][1], R[2][2], t[2]],
        [0.0,     0.0,     0.0,     1.0]
    ]

# LERF OpenGl convention 
Q_RGB_TO_OPTICAL = (0.5, -0.5, 0.5, -0.5)
R_OPTICAL_TO_OPENGL = [[1,0,0],[0,-1,0],[0,0,-1]]

def read_timestamps_csv(path):
    rows = []
    with open(path, "r", newline="") as f:
        for r in csv.DictReader(f):
            sec = int(r["sec"]); nsec = int(r["nanosec"])
            rows.append({"filename": r["filename"], "t": sec + nsec*1e-9})
    return rows

def read_poses_csv(path):
    timestamp, vals = [], []
    with open(path, "r", newline="") as f:
        reader = csv.DictReader(f)
        timestamp_field = "timestamp" if "timestamp" in reader.fieldnames else "timestamp (sec.nanosec)"
        for r in reader:
            t_str = r[timestamp_field].strip()
            if "." in t_str:
                s, ns = t_str.split(".", 1)
                t = int(s) + int(ns[:9].ljust(9,"0"))*1e-9
            else:
                t = float(t_str)
            p = (float(r["pos_x"]), float(r["pos_y"]), float(r["pos_z"]))
            q = (float(r["or_x"]), float(r["or_y"]), float(r["or_z"]), float(r["or_w"]))
            timestamp.append(t); vals.append((p,q))
    # sortiert zurückgeben
    pairs = sorted(zip(timestamp, vals), key=lambda x: x[0])
    return [p[0] for p in pairs], [p[1] for p in pairs]

def read_intrinsics(path):
    with open(path, "r") as f:
        J = json.load(f)
    w = int(J["width"]); h = int(J["height"])
    fx = float(J.get("fx", J.get("fl_x", 0.0)))
    fy = float(J.get("fy", J.get("fl_y", 0.0)))
    cx = float(J.get("cx", 0.0)); cy = float(J.get("cy", 0.0))
    if fx == 0.0 and "K" in J:
        fx, cx, fy, cy = float(J["K"][0]), float(J["K"][2]), float(J["K"][4]), float(J["K"][5])
    return {"w": w, "h": h, "fx": fx, "fy": fy, "cx": cx, "cy": cy}

def nearest_pose(timestamp, vals, t): #based on timestamps
    i = bisect_left(timestamp, t)
    if i == 0:
        return vals[0], abs(timestamp[0]-t)
    if i >= len(timestamp):
        return vals[-1], abs(timestamp[-1]-t)
    dl, dr = abs(t - timestamp[i-1]), abs(timestamp[i] - t)
    return (vals[i-1], dl) if dl <= dr else (vals[i], dr)

def world_to_opengl_from_rgb(p_world_rgb, q_world_rgb):
    R_world_rgb = q_to_R(q_world_rgb)
    R_rgb_opt = q_to_R(Q_RGB_TO_OPTICAL)
    R_world_opt = R_mul(R_world_rgb, R_rgb_opt)
    R_world_gl  = R_mul(R_world_opt, R_OPTICAL_TO_OPENGL)
    return mat4(R_world_gl, p_world_rgb)

def main():
    ap = argparse.ArgumentParser(description="transforms.json generator")
    ap.add_argument("--images_dir", required=True)
    ap.add_argument("--timestamps_csv", required=True)
    ap.add_argument("--poses_csv", required=True)
    ap.add_argument("--intrinsics", required=True)
    ap.add_argument("--out", default="transforms.json")
    ap.add_argument("--max_dt", type=float, default=0.15)
    args = ap.parse_args()

    timestamp_rows = read_timestamps_csv(args.timestamps_csv)
    pose_timestamp, pose_vals = read_poses_csv(args.poses_csv)
    intr = read_intrinsics(args.intrinsics)

    frames, skipped = [], 0
    out_dir = os.path.dirname(os.path.abspath(args.out)) or "."
    rel_images_dir = os.path.relpath(os.path.abspath(args.images_dir), start=out_dir).replace("\\","/")

    for r in timestamp_rows:
        t_img = r["t"]
        (p,q), dt_min = nearest_pose(pose_timestamp, pose_vals, t_img)
        if dt_min > args.max_dt:
            skipped += 1
            continue
        T = world_to_opengl_from_rgb(p, q)
        frames.append({
            "file_path": f"{rel_images_dir}/{r['filename']}",
            "transform_matrix": T
        })

    out_json = {
        "w": intr["w"], "h": intr["h"],
        "fl_x": intr["fx"], "fl_y": intr["fy"],
        "cx": intr["cx"], "cy": intr["cy"],
        "frames": frames
    }

    os.makedirs(out_dir, exist_ok=True)
    with open(args.out, "w") as f:
        json.dump(out_json, f, indent=2)

    print(f"[OK] {args.out} geschrieben")
    print(f"     Frames: {len(frames)} | Übersprungen (|dt| > {args.max_dt}s): {skipped}")

if __name__ == "__main__":
    main()

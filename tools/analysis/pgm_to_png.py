#!/usr/bin/env python3
"""把地图 PGM 转成看得清的 PNG（白=空闲 黑=障碍 灰=未知）。

用法：
    python pgm_to_png.py <map.yaml> [输出.png]
"""

import argparse
import os
import sys


def read_pgm(path):
    with open(path, "rb") as f:
        data = f.read()
    fields, i = [], 0
    while len(fields) < 4:
        while i < len(data) and data[i:i + 1].isspace():
            i += 1
        if data[i:i + 1] == b"#":
            while i < len(data) and data[i:i + 1] != b"\n":
                i += 1
            continue
        j = i
        while j < len(data) and not data[j:j + 1].isspace():
            j += 1
        fields.append(data[i:j])
        i = j
    i += 1
    w, h = int(fields[1]), int(fields[2])
    return w, h, data[i:i + w * h]


def read_yaml(path):
    info = {}
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            if ":" in line:
                k, v = line.split(":", 1)
                info[k.strip()] = v.strip()
    return info


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("yaml")
    ap.add_argument("png", nargs="?")
    ap.add_argument("--dpi", type=int, default=110)
    args = ap.parse_args(argv if argv is not None else sys.argv[1:])

    y = read_yaml(args.yaml)
    base = os.path.splitext(os.path.abspath(args.yaml))[0]
    pgm = os.path.join(os.path.dirname(base), y.get("image", ""))
    out = args.png or (base + ".png")

    w, h, px = read_pgm(pgm)
    occ_th = float(y.get("occupied_thresh", 0.65))
    free_th = float(y.get("free_thresh", 0.196))
    res = float(y.get("resolution", 0.05))
    ox, oy = [float(v) for v in y.get("origin", "[-0, -0, 0]")
              .strip("[]").split(",")[:2]]

    import numpy as np
    img = np.frombuffer(px, dtype=np.uint8).reshape(h, w).astype(np.float32)
    p = (255.0 - img) / 255.0
    # 0=unknown(灰) 1=free(白) 2=occupied(黑)
    cls = np.zeros((h, w), dtype=np.uint8)
    cls[p < free_th] = 1
    cls[p > occ_th] = 2

    rgb = np.zeros((h, w, 3), dtype=np.uint8)
    rgb[cls == 0] = (200, 200, 200)     # unknown 浅灰
    rgb[cls == 1] = (255, 255, 255)     # free 白
    rgb[cls == 2] = (0, 0, 0)           # occupied 黑
    rgb = np.flipud(rgb)                # PGM 行 0 在下方，翻转成正常视角

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(9, 9 * h / w))
    ax.imshow(rgb, interpolation="nearest",
              extent=[ox, ox + w * res, oy, oy + h * res])
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title("{}   {}x{} px   {} m/px".format(
        os.path.basename(pgm), w, h, res), fontsize=10)
    ax.grid(alpha=0.15)
    fig.tight_layout()
    fig.savefig(out, dpi=args.dpi)
    print("saved", out)
    return 0


if __name__ == "__main__":
    sys.exit(main())

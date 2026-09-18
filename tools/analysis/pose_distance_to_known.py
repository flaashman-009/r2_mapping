#!/usr/bin/env python3
"""位姿离"已建图区域"到底有多远？

pose_vs_map.py 只判断位姿落在"空闲/未知/障碍"哪种格子上，但走廊两侧
本来就是未知区 —— 位姿只要偏一格就会从"空闲"翻成"未知"，那个统计会
严重高估"车开进了没建图的地方"。

这个脚本算的是**到最近已知格（空闲或障碍）的实际距离**，才是真正的判据：
    < 0.3 m  相当于在已建图区域内（只是压在边界上）
    > 1.0 m  才是真的开进了没建图的地方

用法： python pose_distance_to_known.py <nav_watch.csv> <map.yaml>
"""

import argparse
import csv
import math
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
    return int(fields[1]), int(fields[2]), data[i:i + int(fields[1]) * int(fields[2])]


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
    ap.add_argument("csv")
    ap.add_argument("yaml")
    args = ap.parse_args(argv if argv is not None else sys.argv[1:])

    import numpy as np

    def chamfer(mask_true):
        """到最近 True 格的距离（格数）。两趟 chamfer，不需要 scipy。"""
        BIG = 1e9
        d = np.where(mask_true, 0.0, BIG)
        hh, ww = d.shape
        for i in range(hh):                      # 正向
            for j in range(ww):
                v = d[i, j]
                if i:
                    v = min(v, d[i - 1, j] + 1.0)
                    if j:
                        v = min(v, d[i - 1, j - 1] + 1.4142)
                    if j + 1 < ww:
                        v = min(v, d[i - 1, j + 1] + 1.4142)
                if j:
                    v = min(v, d[i, j - 1] + 1.0)
                d[i, j] = v
        for i in range(hh - 1, -1, -1):          # 反向
            for j in range(ww - 1, -1, -1):
                v = d[i, j]
                if i + 1 < hh:
                    v = min(v, d[i + 1, j] + 1.0)
                    if j + 1 < ww:
                        v = min(v, d[i + 1, j + 1] + 1.4142)
                    if j:
                        v = min(v, d[i + 1, j - 1] + 1.4142)
                if j + 1 < ww:
                    v = min(v, d[i, j + 1] + 1.0)
                d[i, j] = v
        return d

    y = read_yaml(args.yaml)
    pgm = os.path.join(os.path.dirname(os.path.abspath(args.yaml)), y["image"])
    w, h, px = read_pgm(pgm)
    res = float(y["resolution"])
    ox, oy = [float(v) for v in y["origin"].strip("[]").split(",")[:2]]
    occ_th = float(y.get("occupied_thresh", 0.65))
    free_th = float(y.get("free_thresh", 0.196))

    img = np.frombuffer(px, dtype=np.uint8).reshape(h, w).astype(np.float32)
    p = (255.0 - img) / 255.0
    known = (p < free_th) | (p > occ_th)      # 空闲或障碍 = 已知
    # 到最近"已知格"的距离（米）
    dist = chamfer(known) * res

    rows = list(csv.DictReader(open(args.csv, encoding="utf-8")))
    g = lambda r, k: float(r[k])
    mov = [r for r in rows if abs(g(r, "cmd_vx")) > 0.005]

    def stats(name, rs):
        d = []
        for r in rs:
            gx = int((g(r, "amcl_x") - ox) / res)
            gy = int((g(r, "amcl_y") - oy) / res)
            if 0 <= gx < w and 0 <= gy < h:
                d.append(float(dist[gy, gx]))
        if not d:
            return
        d.sort()
        print("  {:<8} 帧{:5d}  中位 {:.2f} m  p90 {:.2f} m  最大 {:.2f} m".format(
            name, len(d), d[len(d) // 2], d[int(len(d) * .9)], d[-1]))
        for th in (0.3, 0.5, 1.0, 2.0):
            n = sum(1 for x in d if x > th)
            print("      离最近已知格 > {:.1f} m 的帧：{:5d}  ({:4.1f}%)".format(
                th, n, 100.0 * n / len(d)))

    print("=" * 68)
    print(" 位姿到「最近已建图区域」的距离")
    print("=" * 68)
    stats("运动时", mov)
    stats("全程", rows)
    return 0


if __name__ == "__main__":
    sys.exit(main())

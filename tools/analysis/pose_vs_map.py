#!/usr/bin/env python3
"""把记录里的位姿投到地图上，统计车有多少时间走在"没建过图"的区域。

这是判断"AMCL 为什么会迷失"最直接的一条：
  浅灰色（unknown）格子 = 建图时激光没扫到的地方 = 地图里没有墙
  车一旦开进去，扫描就找不到东西可以匹配 → AMCL 必然漂

用法：
    python pose_vs_map.py <nav_watch.csv> <map.yaml>
"""

import argparse
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
    w, h, maxval = int(fields[1]), int(fields[2]), int(fields[3])
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
    ap.add_argument("csv")
    ap.add_argument("yaml")
    args = ap.parse_args(argv if argv is not None else sys.argv[1:])

    y = read_yaml(args.yaml)
    pgm = os.path.join(os.path.dirname(os.path.abspath(args.yaml)),
                       y["image"])
    w, h, px = read_pgm(pgm)
    res = float(y["resolution"])
    ox, oy = [float(v) for v in
              y["origin"].strip("[]").split(",")[:2]]
    occ_th = float(y.get("occupied_thresh", 0.65))
    free_th = float(y.get("free_thresh", 0.196))

    import csv as csvmod
    rows = list(csvmod.DictReader(open(args.csv, encoding="utf-8")))

    def classify(mx, my):
        gx = int((mx - ox) / res)
        gy = int((my - oy) / res)
        if gx < 0 or gy < 0 or gx >= w or gy >= h:
            return "out"
        v = px[gy * w + gx]
        p = (255 - v) / 255.0
        if p > occ_th:
            return "occ"
        if p < free_th:
            return "free"
        return "unk"

    print("=" * 68)
    print(" 位姿 vs 地图：车走在什么区域")
    print("=" * 68)

    moving = [r for r in rows if abs(float(r["cmd_vx"])) > 0.005]
    samples = moving if moving else rows
    tag = "运动期间" if moving else "全程"

    cnt = {"free": 0, "unk": 0, "occ": 0, "out": 0}
    for r in samples:
        cnt[classify(float(r["amcl_x"]), float(r["amcl_y"]))] += 1
    n = sum(cnt.values())
    print("\n[1] {}（{} 帧）AMCL 位姿落在地图的哪类格子上：".format(tag, n))
    for k, name in (("free", "已建图的空闲区"), ("unk", "未知区（没扫到过）"),
                    ("occ", "障碍格（几乎不可能）"), ("out", "地图范围外")):
        print("    {:<16} {:6d} 帧  {:5.1f}%".format(
            name, cnt[k], 100.0 * cnt[k] / max(1, n)))

    # 与残差的关联
    print("\n[2] 分区看残差（残差高 = 点云对不上地图）")
    for k, name in (("free", "已建图空闲区"), ("unk", "未知区")):
        v = [float(r["res_med"]) for r in samples
             if classify(float(r["amcl_x"]), float(r["amcl_y"])) == k
             and float(r["res_ok"]) > 10]
        if v:
            v.sort()
            print("    {:<12} 样本 {:5d}  残差中位 {:.3f} m  "
                  "p90 {:.3f} m".format(name, len(v), v[len(v) // 2],
                                        v[int(len(v) * 0.9)]))

    # 轨迹长度
    if moving:
        d = 0.0
        for a, b in zip(moving, moving[1:]):
            d += math.hypot(float(b["odom_x"]) - float(a["odom_x"]),
                            float(b["odom_y"]) - float(a["odom_y"]))
        print("\n[3] odom 轨迹长度 {:.2f} m   直线距离 {:.2f} m".format(
            d, math.hypot(float(moving[-1]["odom_x"]) - float(moving[0]["odom_x"]),
                          float(moving[-1]["odom_y"]) - float(moving[0]["odom_y"]))))

    print("\n[4] 地图整体质量")
    tot = {"free": 0, "unk": 0, "occ": 0}
    for v in px:
        p = (255 - v) / 255.0
        if p > occ_th:
            tot["occ"] += 1
        elif p < free_th:
            tot["free"] += 1
        else:
            tot["unk"] += 1
    tt = sum(tot.values())
    print("    空闲 {:.1f}%   障碍 {:.1f}%   未知 {:.1f}%".format(
        100.0 * tot["free"] / tt, 100.0 * tot["occ"] / tt,
        100.0 * tot["unk"] / tt))
    return 0


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""把 PGM 地图降采样成字符画，用来快速判断地图结构。

用法：
    python3 map_ascii.py <map.yaml> [--cols 100]

字符含义：
    #  障碍（occupied）
    .  空闲（free）
    (空格) 未知（unknown）

用途：判断地图是不是"一条窄走廊"或"有重复结构"——
这两者都会让 AMCL 更容易跳到错误的位姿上去。
"""

import argparse
import sys


def read_pgm(path):
    with open(path, "rb") as f:
        data = f.read()
    # 解析 P5 头
    fields = []
    i = 0
    while len(fields) < 4:
        # 跳过空白和注释
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
    i += 1  # 单个空白
    magic, w, h, maxval = fields[0], int(fields[1]), int(fields[2]), int(fields[3])
    if magic != b"P5":
        raise SystemExit("只支持 P5 二进制 PGM")
    px = data[i:i + w * h]
    return w, h, maxval, px


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
    ap.add_argument("--cols", type=int, default=100)
    args = ap.parse_args(argv if argv is not None else sys.argv[1:])

    y = read_yaml(args.yaml)
    import os
    pgm = os.path.join(os.path.dirname(os.path.abspath(args.yaml)),
                       y.get("image", ""))
    occ_th = float(y.get("occupied_thresh", 0.65))
    free_th = float(y.get("free_thresh", 0.196))
    negate = int(y.get("negate", 0))
    w, h, maxval, px = read_pgm(pgm)

    # PGM 行 0 在地图底部（origin 是左下角），打印时翻过来
    rows = args.cols
    scale_x = max(1, w // args.cols)
    scale_y = max(1, int(scale_x * 2.0))  # 字符高宽比
    out = []
    for by in range(0, h, scale_y):
        line = []
        for bx in range(0, w, scale_x):
            occ = free = unk = 0
            for yy in range(by, min(by + scale_y, h)):
                base = yy * w
                for xx in range(bx, min(bx + scale_x, w)):
                    v = px[base + xx]
                    p = (255 - v) / 255.0 if not negate else v / 255.0
                    if p > occ_th:
                        occ += 1
                    elif p < free_th:
                        free += 1
                    else:
                        unk += 1
            if occ > 0 and occ >= free:
                line.append("#")
            elif free > 0:
                line.append(".")
            else:
                line.append(" ")
        out.append("".join(line).rstrip())

    print("地图 {}  {}x{} px  分辨率 {} m/px".format(
        os.path.basename(pgm), w, h, y.get("resolution")))
    print("每字符 = {:.2f} m".format(scale_x * float(y.get("resolution", 0.05))))
    print("+" + "-" * args.cols + "+")
    for line in reversed(out):
        print("|" + line.ljust(args.cols) + "|")
    print("+" + "-" * args.cols + "+")
    print("# = 障碍   . = 空闲   空格 = 未知")
    return 0


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""地图评估：分辨率、occupied/free/unknown 比例、墙线厚度估计、地图对比。

用法：
    python3 -m r2_mapping_tools.map_eval maps/room_01.yaml
    ros2 run r2_mapping_tools map_eval maps/room_01.yaml
    ros2 run r2_mapping_tools map_eval maps/room_01.yaml --compare maps/room_02.yaml
    ros2 run r2_mapping_tools map_eval maps/room_01.yaml --png maps/room_01.png

只依赖 numpy 与 PyYAML。PNG 导出需要 Pillow，缺失时自动跳过。
"""

import argparse
import math
import os
import sys
from collections import deque

import numpy as np

try:
    import yaml
except ImportError:  # pragma: no cover
    yaml = None


def parse_pgm(path):
    """读取 P5 二进制 PGM，返回 (width, height, numpy uint8 数组)。"""
    with open(path, "rb") as fh:
        tokens = []
        while len(tokens) < 4:
            line = fh.readline()
            if not line:
                raise ValueError("PGM 头不完整: " + path)
            line = line.split(b"#")[0].strip()
            if line:
                tokens.extend(line.split())
        magic, width, height, maxval = tokens[0], int(tokens[1]), int(tokens[2]), int(tokens[3])
        if magic not in (b"P5", b"P2"):
            raise ValueError("只支持 P5/P2 PGM，实际是 " + magic.decode("ascii", "replace"))
        if maxval != 255:
            raise ValueError("只支持 maxval=255，实际 " + str(maxval))
        if magic == b"P5":
            data = fh.read(width * height)
            image = np.frombuffer(data, dtype=np.uint8)
        else:
            values = []
            for line in fh:
                values.extend(int(v) for v in line.split())
            image = np.asarray(values, dtype=np.uint8)
    if image.size != width * height:
        raise ValueError(
            "PGM 数据长度与尺寸不符：期望 {} 实际 {}".format(width * height, image.size)
        )
    return width, height, image.reshape((height, width))


def load_map(yaml_path):
    if yaml is None:
        raise RuntimeError("缺少 PyYAML，请安装 python3-yaml")

    with open(yaml_path, "r", encoding="utf-8") as fh:
        meta = yaml.safe_load(fh)

    image_name = meta["image"]
    if not os.path.isabs(image_name):
        image_name = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), image_name)

    width, height, image = parse_pgm(image_name)

    info = {
        "yaml": os.path.abspath(yaml_path),
        "image": os.path.abspath(image_name),
        "resolution": float(meta["resolution"]),
        "origin": list(meta.get("origin", [0.0, 0.0, 0.0])),
        "negate": int(meta.get("negate", 0)),
        "occupied_thresh": float(meta.get("occupied_thresh", 0.65)),
        "free_thresh": float(meta.get("free_thresh", 0.196)),
        "width": width,
        "height": height,
        "image": image,
        "image_path": image_name,
    }
    return info


def classify(info):
    image = info["image"].astype(np.float32)
    p = image / 255.0
    if info["negate"]:
        occupancy = p
    else:
        occupancy = 1.0 - p

    occupied = occupancy > info["occupied_thresh"]
    free = occupancy < info["free_thresh"]
    unknown = ~occupied & ~free
    return occupied, free, unknown


def wall_thickness_estimate(occupied):
    """用「occupied ≈ 厚度 × 长度，边界 ≈ 2 × 长度」估算平均墙厚（格）。"""
    if not occupied.any():
        return 0.0
    boundary = np.zeros_like(occupied)
    boundary[1:, :] |= occupied[1:, :] & ~occupied[:-1, :]
    boundary[:-1, :] |= occupied[:-1, :] & ~occupied[1:, :]
    boundary[:, 1:] |= occupied[:, 1:] & ~occupied[:, :-1]
    boundary[:, :-1] |= occupied[:, :-1] & ~occupied[:, 1:]
    edge = int(boundary.sum())
    if edge == 0:
        return float(occupied.sum())
    return 2.0 * float(occupied.sum()) / float(edge)


def count_components(occupied, max_components=100000):
    """4 连通域计数（BFS），只统计 occupied 格子。"""
    height, width = occupied.shape
    visited = np.zeros_like(occupied, dtype=bool)
    rows, cols = np.nonzero(occupied)
    count = 0
    for r0, c0 in zip(rows.tolist(), cols.tolist()):
        if visited[r0, c0]:
            continue
        count += 1
        if count > max_components:
            return count
        queue = deque([(r0, c0)])
        visited[r0, c0] = True
        while queue:
            r, c = queue.popleft()
            for nr, nc in ((r + 1, c), (r - 1, c), (r, c + 1), (r, c - 1)):
                if 0 <= nr < height and 0 <= nc < width:
                    if occupied[nr, nc] and not visited[nr, nc]:
                        visited[nr, nc] = True
                        queue.append((nr, nc))
    return count


def component_sizes(occupied):
    """返回所有 occupied 连通块（4 连通）的尺寸列表，从大到小排序。"""
    height, width = occupied.shape
    visited = np.zeros_like(occupied, dtype=bool)
    rows, cols = np.nonzero(occupied)
    sizes = []
    for r0, c0 in zip(rows.tolist(), cols.tolist()):
        if visited[r0, c0]:
            continue
        size = 0
        queue = deque([(r0, c0)])
        visited[r0, c0] = True
        while queue:
            r, c = queue.popleft()
            size += 1
            for nr, nc in ((r + 1, c), (r - 1, c), (r, c + 1), (r, c - 1)):
                if 0 <= nr < height and 0 <= nc < width:
                    if occupied[nr, nc] and not visited[nr, nc]:
                        visited[nr, nc] = True
                        queue.append((nr, nc))
        sizes.append(size)
    sizes.sort(reverse=True)
    return sizes


def world_extent(info):
    res = info["resolution"]
    ox, oy = info["origin"][0], info["origin"][1]
    return {
        "x_min": ox,
        "y_min": oy,
        "x_max": ox + info["width"] * res,
        "y_max": oy + info["height"] * res,
    }


def align_to_common_grid(a, b):
    """把两张地图的 occupied 掩码对齐到公共的世界网格。"""
    res = min(a["resolution"], b["resolution"])
    ea, eb = world_extent(a), world_extent(b)
    x_min = min(ea["x_min"], eb["x_min"])
    y_min = min(ea["y_min"], eb["y_min"])
    x_max = max(ea["x_max"], eb["x_max"])
    y_max = max(ea["y_max"], eb["y_max"])
    width = int(math.ceil((x_max - x_min) / res))
    height = int(math.ceil((y_max - y_min) / res))

    def resample(info):
        occ, _, _ = classify(info)
        cols = ((np.arange(info["width"]) * info["resolution"]
                 + info["origin"][0] - x_min) / res).astype(np.int64)
        rows = ((np.arange(info["height"]) * info["resolution"]
                 + info["origin"][1] - y_min) / res).astype(np.int64)
        valid_c = (cols >= 0) & (cols < width)
        valid_r = (rows >= 0) & (rows < height)
        out = np.zeros((height, width), dtype=bool)
        if not valid_r.any() or not valid_c.any():
            return out
        sub = occ[np.ix_(valid_r, valid_c)]
        # 分辨率不同时是「多对一」映射，必须用逻辑或累加，
        # 直接赋值只会保留最后一个源格子的值。
        target_rows = rows[valid_r][:, None]
        target_cols = cols[valid_c][None, :]
        np.logical_or.at(out, (target_rows, target_cols), sub)
        return out

    return resample(a), resample(b)


def export_png(info, out_path):
    try:
        from PIL import Image
    except ImportError:
        print("  （未安装 Pillow，跳过 PNG 导出）")
        return None
    Image.fromarray(info["image"], mode="L").save(out_path)
    return out_path


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="地图评估",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("yaml_path", help="地图 YAML 路径")
    parser.add_argument("--compare", default=None, help="对比另一张地图（闭环一致性）")
    parser.add_argument("--png", default=None, help="导出 PNG 便于查看")
    parser.add_argument("--strict", action="store_true",
                        help="unknown > 60%% 或 occupied < 1%% 时返回非零")
    args = parser.parse_args(argv if argv is not None else sys.argv[1:])

    info = load_map(args.yaml_path)
    occupied, free, unknown = classify(info)
    total = float(occupied.size)

    ext = world_extent(info)
    print("")
    print("=" * 62)
    print(" 地图评估：" + os.path.basename(args.yaml_path))
    print("=" * 62)
    print("  文件      : " + info["yaml"])
    print("  图像      : {}  ({} x {} px)".format(
        os.path.basename(info["image_path"]), info["width"], info["height"]))
    print("  分辨率    : {:.4f} m/cell".format(info["resolution"]))
    print("  origin    : [{:.3f}, {:.3f}, {:.3f}]".format(*info["origin"][:3]))
    print("  世界范围  : x [{:.2f}, {:.2f}]  y [{:.2f}, {:.2f}]  ({:.1f} x {:.1f} m)".format(
        ext["x_min"], ext["x_max"], ext["y_min"], ext["y_max"],
        ext["x_max"] - ext["x_min"], ext["y_max"] - ext["y_min"]))
    print("  阈值      : occupied > {:.2f} / free < {:.2f} / negate {}".format(
        info["occupied_thresh"], info["free_thresh"], info["negate"]))

    print("")
    print("  occupied  : {:>10d} 格  ({:5.1f}%)".format(
        int(occupied.sum()), 100.0 * occupied.sum() / total))
    print("  free      : {:>10d} 格  ({:5.1f}%)".format(
        int(free.sum()), 100.0 * free.sum() / total))
    print("  unknown   : {:>10d} 格  ({:5.1f}%)".format(
        int(unknown.sum()), 100.0 * unknown.sum() / total))

    # ROS 地图的三个约定灰度：0 = occupied，254 = free，205 = unknown。
    # 注意 205 对应的占据概率是 (255-205)/255 = 0.196：
    # 只要 free_thresh > 0.196（例如常见模板里的 0.25），
    # 205 就会被判成 free —— unknown 区域会在地图里"消失"。
    raw_image = info["image"]
    grey_unknown = int((raw_image == 205).sum())
    grey_ratio = grey_unknown / total
    classified_unknown_ratio = float(unknown.sum()) / total
    unknown_swallowed = (
        info["negate"] == 0
        and grey_ratio > 0.01
        and classified_unknown_ratio < 0.5 * grey_ratio
    )

    print("")
    print("  原始灰度 205（约定 unknown）: {} 格 ({:.1f}%)".format(
        grey_unknown, 100.0 * grey_ratio))
    if unknown_swallowed:
        print("    ⚠️ 这些格子在当前 free_thresh={:.3f} 下被判成了 free。".format(
            info["free_thresh"]))
        print("       想让 unknown 保留，把 free_thresh 改成 0.196（或更小）。")
    elif info["negate"] == 1:
        print("    （negate=1，无法用灰度判断 unknown，本项跳过）")

    thickness = wall_thickness_estimate(occupied)
    sizes = component_sizes(occupied)
    components = len(sizes)
    print("")
    print("  平均墙厚估计 : {:.2f} 格（{:.3f} m），目标 1-2 格".format(
        thickness, thickness * info["resolution"]))

    # ---- 只在"已探索区域"里统计，否则巨大的未知包围盒会把比例稀释掉 ----
    known = occupied | free
    bbox = None
    if known.any():
        rows_any = np.any(known, axis=0)   # 注意：这里按列/行分开算边界
        cols_any = np.any(known, axis=1)
        c0, c1 = np.where(rows_any)[0][[0, -1]]
        r0, r1 = np.where(cols_any)[0][[0, -1]]
        bbox = (int(r0), int(r1), int(c0), int(c1))

    if bbox is not None:
        r0, r1, c0, c1 = bbox
        sub_occ = occupied[r0:r1 + 1, c0:c1 + 1]
        sub_known = known[r0:r1 + 1, c0:c1 + 1]
        sub_total = float(sub_known.size)
        occ_ratio = float(sub_occ.sum()) / sub_total
        known_ratio = float(sub_known.sum()) / sub_total

        print("")
        print("  [已探索区域]  ← 判断地图好坏要看这一段")
        print("    bbox        : {:.1f} m x {:.1f} m".format(
            (c1 - c0 + 1) * info["resolution"], (r1 - r0 + 1) * info["resolution"]))
        print("    覆盖率      : {:.1f}%（bbox 内已观测格子的比例）".format(100.0 * known_ratio))
        print("    occupied    : {:.1f}%（室内典型 2-5%）".format(100.0 * occ_ratio))
        print("    障碍连通块  : 总计 {} 个，其中 >=50 格的 {} 个（这些才是真墙）".format(
            components, sum(1 for s in sizes if s >= 50)))
        small_list = [s for s in sizes if s < 10]
        occ_total = float(occupied.sum()) or 1.0
        small = (len(small_list), sum(small_list) / occ_total)
        area = (c1 - c0 + 1) * (r1 - r0 + 1) * info["resolution"] ** 2
        density = small[0] / area if area > 0 else 0.0
        occ_density = occ_ratio / (info["resolution"] ** 2)
        print("    碎斑        : {} 个 <10 格的碎块".format(small[0]))
        print("                  占 occupied 的 {:.1f}%".format(100.0 * small[1]))
        print("                  密度 {:.2f} 个/m²   ← 换地图/换面积时看这个".format(density))
        print("    occupied密度: {:.1f} 格/m²".format(occ_density))
    else:
        occ_ratio = 0.0
        small = (0, 0.0)
        density = 0.0

    notes = []
    if bbox is not None and occ_ratio < 0.01:
        notes.append("已探索区域内 occupied 低于 1%：地图几乎没墙，检查 LiDAR 外参与量程")
    # 判断碎斑要用**密度**而不是占比：
    # 占比 = 碎块格子数 / occupied 总格子数，面积一变这个比值就会失真
    # （场地大了 occupied 增长慢于碎块增长，占比天然升高）。
    # 经验值：办公室/实验室这类有桌椅家具的场地，1-2 个/m² 是正常的，
    # 超过 ~4 个/m² 才说明雷达噪声或车体自遮挡明显。
    if bbox is not None and density > 4.0:
        notes.append(
            "碎斑密度偏高（{:.2f} 个/m²，共 {} 个 <10 格碎块）："
            "可能是雷达噪声或车体自遮挡，用 scripts/scan_stats.py 确认".format(
                density, small[0]))
    elif bbox is not None and density > 0.0:
        notes.append(
            "碎斑密度 {:.2f} 个/m²（{} 个）：这个量级通常是环境里的桌椅腿等"
            "真实小障碍，不是噪声".format(density, small[0]))
    if thickness > 4.0:
        notes.append("墙厚 > 4 格：可能有重影（闭环没对上）或障碍膨胀过大")
    if unknown_swallowed:
        notes.append(
            "free_thresh={:.3f} 把 unknown(205) 判成了 free："
            "地图里会出现“凭空多出来的可通行区域”，"
            "建议 free_thresh: 0.196".format(info["free_thresh"])
        )

    if args.compare:
        other = load_map(args.compare)
        if not math.isclose(other["resolution"], info["resolution"], rel_tol=1e-6):
            print("\n  ⚠️ 两张地图分辨率不同（{} vs {}），对比已按较细者重采样".format(
                info["resolution"], other["resolution"]))
        mask_a, mask_b = align_to_common_grid(info, other)
        inter = np.logical_and(mask_a, mask_b).sum()
        union = np.logical_or(mask_a, mask_b).sum()
        iou = float(inter) / float(union) if union else 0.0
        print("\n  [地图对比]")
        print("    对比对象 : " + other["yaml"])
        print("    occupied IoU : {:.3f}".format(iou))
        if iou >= 0.7:
            print("    结论     : 一致性好（闭环可接受）")
        elif iou >= 0.4:
            print("    结论     : 部分重合，检查闭环与里程计标尺")
        else:
            print("    结论     : 基本对不上，先做 Gate A 自检再重建")
            notes.append("地图对比 IoU 过低")

    if args.png:
        out = export_png(info, args.png)
        if out:
            print("\n  PNG 导出: " + out)

    if notes:
        print("\n  注意：")
        for note in notes:
            print("    ! " + note)
    else:
        print("\n  统计项均在合理范围。")
    print("  （比例合理 ≠ 地图正确；闭环重合仍需人工在 RViz 里看）")
    print("=" * 62)

    if args.strict and notes:
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())

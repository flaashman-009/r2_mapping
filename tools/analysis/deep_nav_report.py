#!/usr/bin/env python3
"""对 nav_watch 记录做深入分析：尖峰、协方差、转向平滑度、航向漂移。"""

import csv
import math
import statistics
import sys


def wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def main(argv):
    path = argv[0]
    rows = list(csv.DictReader(open(path, encoding="utf-8")))
    g = lambda r, k: float(r[k])
    t = [g(r, "t") for r in rows]
    t0 = t[0]

    print("=" * 72)
    print(" 深入分析：{}".format(path))
    print("=" * 72)

    # ---------------- 转向平滑度（现在看 linear.y）----------------
    print("\n[1] 转向平滑度（现在转向走 linear.y，所以看它）")
    for name, key, scale in (("指令 linear.y", "cmd_steer", 1000.0),
                             ("实际转角", "vel_steer", 1.0)):
        v = [g(r, key) * scale for r in rows]
        chg = [abs(b - a) for a, b in zip(v, v[1:]) if abs(b - a) > 0.5]
        span = t[-1] - t[0]
        print("    {:<12} 范围 [{:+.1f}, {:+.1f}] 度   变化>0.5° {} 次 = {:.2f} 次/秒".format(
            name, min(v), max(v), len(chg), len(chg) / span))
        if chg:
            print("                 单步变化 中位 {:.1f}°  最大 {:.1f}°".format(
                statistics.median(chg), max(chg)))
        sign = sum(1 for a, b in zip(v, v[1:]) if a * b < -1 and abs(a) > 3)
        print("                 符号翻转 {} 次".format(sign))

    # ---------------- 残差尖峰 ----------------
    print("\n[2] 残差异常段（中位数 > 0.3 m 的连续区间）")
    bad = [i for i in range(len(rows)) if g(rows[i], "res_med") > 0.3]
    if not bad:
        print("    没有")
    else:
        segs = []
        s = bad[0]
        for a, b in zip(bad, bad[1:]):
            if b - a > 20:
                segs.append((s, a))
                s = b
        segs.append((s, bad[-1]))
        for s, e in segs:
            print("    t={:6.0f} ~ {:6.0f} s ({:4.0f}s)  残差中位 {:.2f}  最大 {:.2f}".format(
                t[s] - t0, t[e] - t0, t[e] - t[s],
                statistics.median([g(rows[i], "res_med") for i in range(s, e + 1)]),
                max(g(rows[i], "res_med") for i in range(s, e + 1))))

    # ---------------- 协方差 ----------------
    print("\n[3] AMCL 协方差（定位不确定度）")
    cov = [g(r, "amcl_cov") for r in rows]
    print("    中位 {:.3f}   p90 {:.3f}   最大 {:.3f}".format(
        statistics.median(cov), sorted(cov)[int(len(cov) * 0.9)], max(cov)))
    hi = [i for i in range(len(rows)) if cov[i] > 5.0]
    print("    协方差 > 5 的帧数：{} （占 {:.1f}%）".format(
        len(hi), 100.0 * len(hi) / len(rows)))
    for i in hi[:8]:
        print("      t={:6.0f}s  cov={:.2f}".format(t[i] - t0, cov[i]))

    # ---------------- map->odom 跳变分布 ----------------
    print("\n[4] map->odom 跳变幅度分布")
    mx = [g(r, "mo_x") for r in rows]
    my = [g(r, "mo_y") for r in rows]
    myaw = [g(r, "mo_yaw") for r in rows]
    d, dy = [], []
    for i in range(1, len(rows)):
        d.append(math.hypot(mx[i] - mx[i - 1], my[i] - my[i - 1]))
        dy.append(abs(math.degrees(wrap(myaw[i] - myaw[i - 1]))))
    d.sort()
    dy.sort()
    print("    单步位移 中位 {:.3f} m   p90 {:.3f}   p99 {:.3f}   最大 {:.3f}".format(
        statistics.median(d), d[int(len(d) * .9)], d[int(len(d) * .99)], d[-1]))
    print("    单步转角 中位 {:.2f}°  p90 {:.2f}°  p99 {:.2f}°  最大 {:.2f}°".format(
        statistics.median(dy), dy[int(len(dy) * .9)], dy[int(len(dy) * .99)], dy[-1]))
    for th in (0.3, 0.5, 1.0):
        n = sum(1 for x in d if x > th)
        print("    位移 > {:.1f} m 的次数：{}".format(th, n))

    # ---------------- 解卷绕后的 map->odom yaw ----------------
    print("\n[5] map->odom 的 yaw（解卷绕，看累计漂移）")
    un = [myaw[0]]
    for v in myaw[1:]:
        un.append(un[-1] + wrap(v - un[-1]))
    print("    起点 {:.1f}°   终点 {:.1f}°   累计变化 {:.1f}°".format(
        math.degrees(un[0]), math.degrees(un[-1]),
        math.degrees(un[-1] - un[0])))
    print("    范围 [{:.1f}°, {:.1f}°]".format(
        math.degrees(min(un)), math.degrees(max(un))))

    # ---------------- 车实际转向了多少 ----------------
    print("\n[6] 里程计航向变化（车实际转了多少）")
    oy = [g(r, "odom_yaw") for r in rows]
    oyun = [oy[0]]
    for v in oy[1:]:
        oyun.append(oyun[-1] + wrap(v - oyun[-1]))
    print("    odom yaw 累计变化 {:.1f}°".format(math.degrees(oyun[-1] - oyun[0])))
    dist = 0.0
    ox, oyy = [g(r, "odom_x") for r in rows], [g(r, "odom_y") for r in rows]
    for i in range(1, len(rows)):
        dist += math.hypot(ox[i] - ox[i - 1], oyy[i] - oyy[i - 1])
    print("    odom 轨迹长度 {:.1f} m   直线距离 {:.1f} m".format(
        dist, math.hypot(ox[-1] - ox[0], oyy[-1] - oyy[0])))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))

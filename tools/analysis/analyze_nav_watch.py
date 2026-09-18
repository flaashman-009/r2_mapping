#!/usr/bin/env python3
"""分析 nav_watch.py 记录下来的 CSV，定位"从哪一刻开始漂"。

用法：
    python analyze_nav_watch.py <csv> [--png out.png]
"""

import argparse
import csv
import math
import statistics
import sys


def col(rows, name):
    return [float(r[name]) for r in rows]


def deg(x):
    return math.degrees(x)


def wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def rmse_imu_odom(rows):
    """IMU 航向与 EKF 航向的偏差（判断融合有没有跑飞）。"""
    d = []
    for r in rows:
        d.append(abs(deg(wrap(float(r["odom_yaw"]) - float(r["imu_yaw"])))))
    return d


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--png")
    args = ap.parse_args(argv if argv is not None else sys.argv[1:])

    rows = list(csv.DictReader(open(args.csv, encoding="utf-8")))
    if not rows:
        print("CSV 是空的")
        return 1
    t = col(rows, "t")
    n = len(rows)
    print("=" * 74)
    print(" 记录分析：{}".format(args.csv))
    print("=" * 74)
    print("  帧数 {}   时长 {:.0f} s".format(n, t[-1] - t[0]))

    # ---------------------------------------------------------- 运动区间
    vx = col(rows, "cmd_vx")
    moving = [i for i in range(n) if abs(vx[i]) > 0.005]
    print("\n[1] 运动区间")
    if moving:
        print("  有速度指令：{} 帧（{:.0f} ~ {:.0f} s）".format(
            len(moving), t[moving[0]], t[moving[-1]]))
        print("  |vx| 最大 {:.3f} m/s   中位 {:.3f} m/s".format(
            max(abs(v) for v in vx), statistics.median(
                [abs(v) for v in vx if abs(v) > 0.005])))
    else:
        print("  没有速度指令 —— 车没动过？")

    # ---------------------------------------------------------- 转向
    cw = col(rows, "cmd_w")
    cy = col(rows, "cmd_steer")
    vs = col(rows, "vel_steer")
    print("\n[2] 转向通道")
    print("  /cmd_vel.linear.y  非零帧 {:6d}   范围 [{:+.4f},{:+.4f}]".format(
        sum(1 for v in cy if abs(v) > 1e-9), min(cy), max(cy)))
    print("  /cmd_vel.angular.z 非零帧 {:6d}   范围 [{:+.4f},{:+.4f}]".format(
        sum(1 for v in cw if abs(v) > 1e-9), min(cw), max(cw)))
    print("  /vel_raw.linear.y  非零帧 {:6d}   范围 [{:+.1f},{:+.1f}] 度".format(
        sum(1 for v in vs if abs(v) > 1e-9), min(vs), max(vs)))

    chg = sum(1 for a, b in zip(cw, cw[1:]) if abs(b - a) > 1e-3)
    span = max(1e-6, t[-1] - t[0])
    print("  angular.z 变化次数 {}  = {:.2f} 次/秒  ← 舵机抖动的直接来源".format(
        chg, chg / span))
    sign_flips = sum(1 for a, b in zip(cw, cw[1:]) if a * b < -1e-6)
    print("  angular.z 符号翻转 {} 次  ← 左右来回打方向".format(sign_flips))

    # ---------------------------------------------------------- 残差
    rm = col(rows, "res_med")
    rp = col(rows, "res_p90")
    rok = col(rows, "res_ok")
    valid = [i for i in range(n) if rok[i] > 10]
    print("\n[3] 点云-地图残差（关键指标）")
    if valid:
        v = [rm[i] for i in valid]
        print("  有效样本 {}   中位数 {:.3f} m   p90 {:.3f} m   最大 {:.3f} m".format(
            len(valid), statistics.median(v),
            sorted(v)[int(len(v) * 0.9)], max(v)))
        # 按时间分段看趋势
        print("  分段趋势（每段 60 秒）：")
        t0 = t[valid[0]]
        seg = 60.0
        while t0 < t[valid[-1]]:
            s = [rm[i] for i in valid if t0 <= t[i] < t0 + seg]
            if s:
                print("    t={:5.0f}-{:5.0f}s  残差中位 {:.3f} m  "
                      "最大 {:.3f} m  (n={})".format(
                          t0 - t[0], t0 - t[0] + seg,
                          statistics.median(s), max(s), len(s)))
            t0 += seg
        # 首次超过阈值
        for th in (0.25, 0.35, 0.50):
            hit = [t[i] - t[0] for i in valid if rm[i] > th]
            print("  首次残差 > {:.2f} m : {}".format(
                th, "{:.0f} s".format(hit[0]) if hit else "从未"))
    else:
        print("  没有有效样本 —— 残差没算出来")

    # ---------------------------------------------------------- map->odom
    mx, my, mw = col(rows, "mo_x"), col(rows, "mo_y"), col(rows, "mo_yaw")
    jumps = []
    for i in range(1, n):
        d = math.hypot(mx[i] - mx[i - 1], my[i] - my[i - 1])
        dy = abs(deg(wrap(mw[i] - mw[i - 1])))
        if d > 0.15 or dy > 5.0:
            jumps.append((t[i] - t[0], d, dy))
    print("\n[4] map->odom（AMCL 修正量）")
    print("  起点 ({:+.2f},{:+.2f}) {:+.1f}°   终点 ({:+.2f},{:+.2f}) {:+.1f}°".format(
        mx[0], my[0], deg(mw[0]), mx[-1], my[-1], deg(mw[-1])))
    print("  跳变次数（>0.15m 或 >5°）: {}".format(len(jumps)))
    for tt, d, dy in jumps[:12]:
        print("    t={:6.1f}s  位移 {:+.3f} m  转角 {:+.1f}°".format(tt, d, dy))
    dx = max(mx) - min(mx)
    dyv = max(my) - min(my)
    print("  map->odom 跨度: Δx={:.2f} m  Δy={:.2f} m  Δyaw={:.1f}°".format(
        dx, dyv, deg(max(mw) - min(mw))))

    # ---------------------------------------------------------- 定位漂移
    ax, ay = col(rows, "amcl_x"), col(rows, "amcl_y")
    ox, oy = col(rows, "odom_x"), col(rows, "odom_y")
    cov = col(rows, "amcl_cov")
    print("\n[5] 定位漂移程度")
    print("  AMCL 协方差：中位 {:.5f}  最大 {:.5f}".format(
        statistics.median(cov), max(cov)))
    # AMCL 位姿 与 odom 位姿 的差（车载坐标系下不可直接比，改看 yaw）
    ayaw, oyaw = col(rows, "amcl_yaw"), col(rows, "odom_yaw")
    d1 = abs(deg(wrap(ayaw[0] - oyaw[0])))
    d2 = abs(deg(wrap(ayaw[-1] - oyaw[-1])))
    print("  amcl_yaw - odom_yaw : 起点 {:.1f}°  终点 {:.1f}°".format(d1, d2))

    # ---------------------------------------------------------- 航向源
    iy = col(rows, "imu_yaw")
    imu_steps = []
    for i in range(1, n):
        imu_steps.append(abs(deg(wrap(iy[i] - iy[i - 1]))))
    print("\n[6] 航向源 (/imu/data, madgwick)")
    print("  单步变化 中位 {:.3f}°  最大 {:.3f}°".format(
        statistics.median(imu_steps), max(imu_steps)))
    big = [i for i, s in enumerate(imu_steps) if s > 2.0]
    print("  >2° 的跳变 {} 次".format(len(big)))
    for i in big[:10]:
        print("    t={:6.1f}s  跳 {:.2f}°".format(t[i + 1] - t[0], imu_steps[i]))
    dy2 = rmse_imu_odom(rows)
    print("  |odom_yaw - imu_yaw| 中位 {:.2f}°  最大 {:.2f}°".format(
        statistics.median(dy2), max(dy2)))

    # ---------------------------------------------------------- 绘图
    if args.png:
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
            tt = [x - t[0] for x in t]
            fig, axes = plt.subplots(5, 1, figsize=(14, 14), sharex=True)
            axes[0].plot(tt, ox, label="odom x")
            axes[0].plot(tt, ax, label="amcl x")
            axes[0].plot(tt, oy, label="odom y")
            axes[0].plot(tt, ay, label="amcl y")
            axes[0].set_ylabel("position (m)")
            axes[0].legend(fontsize=8)
            axes[0].grid(alpha=.3)

            axes[1].plot(tt, [deg(v) for v in oyaw], label="odom yaw")
            axes[1].plot(tt, [deg(v) for v in ayaw], label="amcl yaw")
            axes[1].plot(tt, [deg(v) for v in iy], label="imu yaw",
                         alpha=.6)
            axes[1].set_ylabel("yaw (deg)")
            axes[1].legend(fontsize=8)
            axes[1].grid(alpha=.3)

            axes[2].plot(tt, rm, label="residual median")
            axes[2].plot(tt, rp, label="residual p90", alpha=.6)
            for th in (0.25, 0.35, 0.5):
                axes[2].axhline(th, ls="--", lw=.8, color="r")
            axes[2].set_ylabel("scan-map residual (m)")
            axes[2].legend(fontsize=8)
            axes[2].grid(alpha=.3)

            axes[3].plot(tt, mx, label="map->odom x")
            axes[3].plot(tt, my, label="map->odom y")
            axes[3].set_ylabel("map->odom (m)")
            axes[3].legend(fontsize=8)
            axes[3].grid(alpha=.3)

            axes[4].plot(tt, cw, label="cmd angular.z", lw=.8)
            axes[4].plot(tt, cy, label="cmd linear.y", lw=.8)
            axes[4].plot(tt, vx, label="cmd vx", alpha=.5)
            axes[4].set_ylabel("cmd")
            axes[4].set_xlabel("t (s)")
            axes[4].legend(fontsize=8)
            axes[4].grid(alpha=.3)
            fig.tight_layout()
            fig.savefig(args.png, dpi=110)
            print("\n图已保存：{}".format(args.png))
        except Exception as exc:
            print("\n绘图失败：{}".format(exc))
    return 0


if __name__ == "__main__":
    sys.exit(main())

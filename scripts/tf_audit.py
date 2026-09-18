#!/usr/bin/env python3
"""TF 体检：找出谁在发哪条 TF，并检测"两个节点抢发同一条 TF"。

为什么需要它：
  如果 base_node_R2 和 ekf_filter_node **同时**广播 odom->base_footprint，
  TF 树会在这两个值之间来回跳，AMCL 会以为机器人瞬间瞬移，
  表现就是"点云和图突然完全错开、定位彻底迷失"。

用法（硬件已经跑起来时）：

    python3 ~/r2_mapping/scripts/tf_audit.py                # 采样 20 秒
    python3 ~/r2_mapping/scripts/tf_audit.py --duration 60
    python3 ~/r2_mapping/scripts/tf_audit.py --moving       # 车在动时测

输出：
  1. 每条 TF 边（parent->child）的发布频率；
  2. 每条边在采样期内的空间跨度（静止时应该接近 0）；
  3. odom->base_footprint 的单步跳变统计 —— 跳变大 = TF 打架。
"""

import argparse
import math
import statistics
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from tf2_msgs.msg import TFMessage


def quat_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class TfAudit(Node):

    def __init__(self):
        super().__init__("tf_audit")
        self.t0 = time.monotonic()
        # edge -> {count, stamps, xs, ys, yaws}
        self.edges = {}
        self.static_edges = {}

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=200,
        )
        self.create_subscription(TFMessage, "/tf", self._on_tf, qos)
        self.create_subscription(
            TFMessage, "/tf_static", self._on_tf_static,
            QoSProfile(depth=100,
                       durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=QoSReliabilityPolicy.RELIABLE))

    def _stamp(self):
        return time.monotonic() - self.t0

    def _on_tf(self, msg):
        t = self._stamp()
        for tr in msg.transforms:
            key = "{} -> {}".format(tr.header.frame_id,
                                    tr.child_frame_id)
            rec = self.edges.setdefault(key, {"t": [], "x": [], "y": [],
                                              "yaw": []})
            rec["t"].append(t)
            rec["x"].append(tr.transform.translation.x)
            rec["y"].append(tr.transform.translation.y)
            rec["yaw"].append(quat_to_yaw(tr.transform.rotation))

    def _on_tf_static(self, msg):
        for tr in msg.transforms:
            key = "{} -> {}".format(tr.header.frame_id,
                                    tr.child_frame_id)
            rec = self.static_edges.setdefault(key, {"x": set(), "y": set(),
                                                     "yaw": set(), "n": 0})
            rec["n"] += 1
            rec["x"].add(round(tr.transform.translation.x, 4))
            rec["y"].add(round(tr.transform.translation.y, 4))
            rec["yaw"].add(round(math.degrees(
                quat_to_yaw(tr.transform.rotation)), 2))


def wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="TF 冲突体检",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("--duration", type=float, default=20.0)
    parser.add_argument("--jump", type=float, default=0.05,
                        help="单步位移超过多少米算一次跳变")
    args = parser.parse_args(argv if argv is not None else sys.argv[1:])

    rclpy.init()
    node = TfAudit()

    print("=" * 66)
    print(" TF 体检")
    print("=" * 66)

    # ---------------------------------------------------------- 发布者
    print("\n[1] 谁在发 /tf —— 从 ROS 图里查（这一条最直接）")
    try:
        infos = node.get_publishers_info_by_topic("/tf")
    except Exception as exc:
        infos = []
        print("    查询失败：{}".format(exc))
    for info in infos:
        ns = info.node_namespace.rstrip("/")
        print("    - {}{}".format(ns, info.node_name))
    print("    /tf 发布者共 {} 个".format(len(infos)))

    # ---------------------------------------------------------- 采样
    print("\n[2] 采样 {:.0f} 秒 ...".format(args.duration))
    end = time.monotonic() + args.duration
    while rclpy.ok() and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)

    # ---------------------------------------------------------- 动态边
    print("\n[3] /tf 上的边（每条边的发布频率和空间跨度）")
    if not node.edges:
        print("    （没有收到 /tf —— 硬件没跑？）")
    for key in sorted(node.edges):
        rec = node.edges[key]
        n = len(rec["t"])
        span = max(1e-6, rec["t"][-1] - rec["t"][0])
        hz = n / span
        dx = max(rec["x"]) - min(rec["x"])
        dy = max(rec["y"]) - min(rec["y"])
        print("    {:<28} {:6.1f} Hz  帧{:5d}  Δx={:.3f} Δy={:.3f}".format(
            key, hz, n, dx, dy))

    # ---------------------------------------------------------- 跳变
    print("\n[4] 关键边的单步跳变（判断 TF 有没有打架）")
    targets = [k for k in node.edges
               if "odom" in k and ("base_footprint" in k or "base_link" in k)]
    if not targets:
        print("    没找到 odom -> base_* 的边")
    for key in targets:
        rec = node.edges[key]
        jumps = []
        yaw_jumps = []
        for i in range(1, len(rec["t"])):
            d = math.hypot(rec["x"][i] - rec["x"][i - 1],
                           rec["y"][i] - rec["y"][i - 1])
            jumps.append(d)
            yaw_jumps.append(abs(wrap(rec["yaw"][i] - rec["yaw"][i - 1])))
        if not jumps:
            continue
        big_xy = [d for d in jumps if d > args.jump]
        big_yaw = [math.degrees(y) for y in yaw_jumps
                   if math.degrees(y) > 5.0]
        print("    {}".format(key))
        print("      单步位移  中位数 {:.4f} m  最大 {:.4f} m".format(
            statistics.median(jumps), max(jumps)))
        print("      单步转角  中位数 {:.2f}°  最大 {:.2f}°".format(
            math.degrees(statistics.median(yaw_jumps)),
            math.degrees(max(yaw_jumps))))
        print("      >{:.2f}m 的跳变 {} 次；>5° 的跳变 {} 次".format(
            args.jump, len(big_xy), len(big_yaw)))

    # ---------------------------------------------------------- 静态边
    print("\n[5] /tf_static 上的边")
    if not node.static_edges:
        print("    （没有收到 /tf_static）")
    for key in sorted(node.static_edges):
        rec = node.static_edges[key]
        flag = ""
        if len(rec["x"]) > 1:
            flag = "   <<< 同一对 frame 有多个不同值！"
        print("    {:<28} x={} y={} yaw={}°{}".format(
            key, sorted(rec["x"]), sorted(rec["y"]), sorted(rec["yaw"]), flag))

    print("\n" + "=" * 66)
    print(" 判读")
    print("=" * 66)
    n_odom_edge = sum(1 for k in node.edges
                      if "odom" in k and "base_footprint" in k)
    if n_odom_edge and len(infos) > 1:
        print("  /tf 有 {} 个发布者。若其中两个都在发 odom->base_footprint，".format(
            len(infos)))
        print("  就是 TF 打架 —— 直接查 [1] 里的节点名，关掉 base_node 的 TF 广播。")
    for key in targets:
        rec = node.edges[key]
        if len(rec["t"]) < 3:
            continue
        worst = max(abs(wrap(rec["yaw"][i] - rec["yaw"][i - 1]))
                    for i in range(1, len(rec["t"])))
        if math.degrees(worst) > 5.0:
            print("  {} 出现 {:.1f}° 的单步跳变 —— 航向不连续。".format(
                key, math.degrees(worst)))
    print("  静止时 Δx/Δy 应 < 0.01 m；行驶时应连续无跳。")
    print("")

    try:
        node.destroy_node()
    except Exception:
        pass
    if rclpy.ok():
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())

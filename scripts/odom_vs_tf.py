#!/usr/bin/env python3
"""判断 AMCL/建图吃的是哪一份里程计。

车上同时有两个节点在发 /tf：
    /base_node        —— 原厂里程计，航向用"转向角模型"推算（已知是错的）
    /ekf_filter_node  —— 我们改过的 EKF，航向来自 IMU

AMCL 和 slam_toolbox **不读 /odom 话题，只读 TF 里的 odom->base_footprint**。
所以必须搞清楚这条 TF 到底是谁发的。

做法：同时采 /tf(odom->base_footprint)、/odom、/odom_raw 三路数据，
把它们的时间戳和数值对齐比较，看 TF 跟哪一个吻合。

用法：
    python3 ~/r2_mapping/scripts/odom_vs_tf.py --duration 15
    # 最好让车动起来（推着走或遥控慢速前进），静止时三者都是 0，分不出来
"""

import argparse
import math
import statistics
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from tf2_msgs.msg import TFMessage


def quat_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


class OdomVsTf(Node):

    def __init__(self, target_parent, target_child):
        super().__init__("odom_vs_tf")
        self.parent = target_parent
        self.child = target_child
        self.t0 = time.monotonic()
        self.tf = []      # (t, x, y, yaw)
        self.odom = []    # (t, x, y, yaw)
        self.raw = []     # (t, x, y, yaw)

        qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=500)
        self.create_subscription(TFMessage, "/tf", self._on_tf, qos)
        self.create_subscription(Odometry, "/odom", self._on_odom, qos)
        self.create_subscription(Odometry, "/odom_raw", self._on_raw, qos)

    def _stamp(self):
        return time.monotonic() - self.t0

    def _on_tf(self, msg):
        for tr in msg.transforms:
            if (tr.header.frame_id == self.parent
                    and tr.child_frame_id == self.child):
                self.tf.append((self._stamp(),
                                tr.transform.translation.x,
                                tr.transform.translation.y,
                                quat_to_yaw(tr.transform.rotation)))

    @staticmethod
    def _pose(msg, t):
        p = msg.pose.pose
        return (t, p.position.x, p.position.y, quat_to_yaw(p.orientation))

    def _on_odom(self, msg):
        self.odom.append(self._pose(msg, self._stamp()))

    def _on_raw(self, msg):
        self.raw.append(self._pose(msg, self._stamp()))


def match_error(a, b, tol=0.15):
    """b 里时间最接近 a 的样本，返回 (匹配数, 位置差中位数, 转角差中位数)。"""
    if not a or not b:
        return 0, float("nan"), float("nan")
    dp, dy = [], []
    n = 0
    j = 0
    for t, x, y, yaw in a:
        best = None
        while j < len(b) and b[j][0] < t - tol:
            j += 1
        k = j
        while k < len(b) and b[k][0] <= t + tol:
            d = abs(b[k][0] - t)
            if best is None or d < best[0]:
                best = (d, b[k])
            k += 1
        if best is None:
            continue
        n += 1
        _, (_, bx, by, byaw) = best
        dp.append(math.hypot(x - bx, y - by))
        dy.append(abs(wrap(yaw - byaw)))
    if not n:
        return 0, float("nan"), float("nan")
    return n, statistics.median(dp), statistics.median(dy)


def rate(samples):
    if len(samples) < 2:
        return 0.0
    span = max(1e-6, samples[-1][0] - samples[0][0])
    return len(samples) / span


def main(argv=None):
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("--duration", type=float, default=15.0)
    parser.add_argument("--parent", default="odom")
    parser.add_argument("--child", default="base_footprint")
    args = parser.parse_args(argv if argv is not None else sys.argv[1:])

    rclpy.init()
    node = OdomVsTf(args.parent, args.child)

    print("=" * 66)
    print(" AMCL 吃的是哪份里程计？")
    print("=" * 66)
    print("\n采 {:.0f} 秒。**最好让车动起来**（推车或遥控慢速前进），".format(
        args.duration))
    print("静止时三路都是 0，区分不出来。\n")

    end = time.monotonic() + args.duration
    while rclpy.ok() and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)

    print("[频率]")
    print("    /tf  {}->{}   {:5.1f} Hz  ({} 帧)".format(
        args.parent, args.child, rate(node.tf), len(node.tf)))
    print("    /odom（EKF）        {:5.1f} Hz  ({} 帧)".format(
        rate(node.odom), len(node.odom)))
    print("    /odom_raw（原厂）    {:5.1f} Hz  ({} 帧)".format(
        rate(node.raw), len(node.raw)))

    n1, d1, y1 = match_error(node.tf, node.odom)
    n2, d2, y2 = match_error(node.tf, node.raw)

    print("\n[TF 与两个里程计源的吻合度]")
    print("    TF vs /odom      匹配 {:4d} 对 | 位置差中位数 {:.4f} m | "
          "转角差中位数 {:.3f}°".format(n1, d1, math.degrees(y1)))
    print("    TF vs /odom_raw  匹配 {:4d} 对 | 位置差中位数 {:.4f} m | "
          "转角差中位数 {:.3f}°".format(n2, d2, math.degrees(y2)))

    print("\n[绝对量级]")
    for name, s in (("/tf", node.tf), ("/odom", node.odom),
                    ("/odom_raw", node.raw)):
        if s:
            xs = [v[1] for v in s]
            ys = [v[2] for v in s]
            yaws = [v[3] for v in s]
            print("    {:<11} x [{:+.3f},{:+.3f}]  y [{:+.3f},{:+.3f}]  "
                  "yaw [{:+.2f}°, {:+.2f}°]".format(
                      name, min(xs), max(xs), min(ys), max(ys),
                      math.degrees(min(yaws)), math.degrees(max(yaws))))

    print("\n" + "=" * 66)
    print(" 判读")
    print("=" * 66)
    if not node.tf:
        print("  没有收到 {}->{} 的 TF。".format(args.parent, args.child))
    elif len(node.tf) > 0:
        if node.odom and node.raw:
            if y1 < y2 / 2 and y1 < math.radians(1.0):
                print("  TF 更接近 /odom（EKF，航向来自 IMU）—— 这是我们想要的。")
            elif y2 < y1 / 2 and y2 < math.radians(1.0):
                print("  >>> TF 更接近 /odom_raw（原厂，航向用转向角模型推算）。")
                print("      说明 EKF 的修正**没有进入 TF**，AMCL/建图吃的是错的那份。")
            else:
                print("  TF 跟两个源都不完全吻合 —— 可能两个节点都在发同一条 TF，")
                print("      TF 缓冲里两份数据交替，取变换时来回插值。")
        tf_hz = rate(node.tf)
        if 20.0 < tf_hz < 45.0:
            print("  TF 频率 {:.1f} Hz 偏高：像是两个发布者叠加".format(tf_hz))
        elif tf_hz < 15.0:
            print("  TF 频率 {:.1f} Hz：只有一个发布者在工作".format(tf_hz))
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

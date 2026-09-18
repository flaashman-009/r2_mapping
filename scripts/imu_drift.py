#!/usr/bin/env python3
"""测量静止状态下的航向漂移率。

同时订阅四个航向来源，各自算漂移速率（度/分钟）：

  /odom            EKF 融合输出      ← SLAM 用的就是它（航向来自 IMU）
  /odom_raw        原厂 base_node    ← **航向来自转向角模型**
  /imu/data        madgwick 滤波输出 ← EKF 的航向输入
  /imu/yaw_deg     底盘板载融合（这台车上没有，会显示无数据）

**静止时**用来测 IMU 的漂移率。

**行驶时**用来测里程计模型的误差 —— 这是本工具最有价值的用法：

  /imu/data 的航向是真值（IMU 直接测量）
  /odom_raw 的航向是 base_node 用"转向角 + 轴距"算出来的

  两者之差 = 里程计模型的误差。
  直行时如果 /imu/data 说没转、/odom_raw 却转了，就说明
  base_node 的航向模型没反映车的真实运动（机械偏差造成的弯曲）。

用法（车要静止、水平放在地面上）：
    python3 imu_drift.py                 # 默认测 120 秒
    python3 imu_drift.py --duration 300  # 测 5 分钟，漂移率更准

判读：
  < 0.5 度/分钟   良好，20 分钟建图累计 < 10°
  0.5 - 2 度/分钟 偏大，建图会明显扭
  > 2 度/分钟     必须修（改 use_mag、换融合源、或提高 SLAM 对激光的信任）
"""

import argparse
import math
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64


def quat_to_yaw(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class ImuDrift(Node):

    def __init__(self):
        super().__init__("imu_drift")
        self.series = {"odom": [], "imu_data": [], "yaw_deg": []}
        self.t0 = time.monotonic()
        self.series["odom_raw"] = []

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=50,
        )
        self.create_subscription(Odometry, "/odom", self._on_odom, qos)
        self.create_subscription(Odometry, "/odom_raw", self._on_odom_raw, qos)
        self.create_subscription(Imu, "/imu/data", self._on_imu, qos)
        self.create_subscription(Float64, "/imu/yaw_deg", self._on_yaw_deg, qos)

    def _stamp(self):
        return time.monotonic() - self.t0

    def _on_odom(self, msg):
        q = msg.pose.pose.orientation
        self.series["odom"].append((self._stamp(), math.degrees(quat_to_yaw(q.x, q.y, q.z, q.w))))

    def _on_odom_raw(self, msg):
        q = msg.pose.pose.orientation
        self.series["odom_raw"].append(
            (self._stamp(), math.degrees(quat_to_yaw(q.x, q.y, q.z, q.w))))

    def _on_imu(self, msg):
        q = msg.orientation
        self.series["imu_data"].append(
            (self._stamp(), math.degrees(quat_to_yaw(q.x, q.y, q.z, q.w))))

    def _on_yaw_deg(self, msg):
        self.series["yaw_deg"].append((self._stamp(), float(msg.data)))


def unwrap_deg(values):
    """把跨越 ±180° 的角度序列解卷绕，还原单调变化。"""
    out = [values[0]]
    for v in values[1:]:
        d = v - out[-1]
        while d > 180.0:
            d -= 360.0
        while d < -180.0:
            d += 360.0
        out.append(out[-1] + d)
    return out


def linear_slope(t, y):
    """最小二乘斜率，返回 (斜率[度/秒], 截距)。"""
    n = len(t)
    if n < 2:
        return 0.0, 0.0
    mt = sum(t) / n
    my = sum(y) / n
    num = sum((ti - mt) * (yi - my) for ti, yi in zip(t, y))
    den = sum((ti - mt) ** 2 for ti in t)
    if den == 0:
        return 0.0, 0.0
    slope = num / den
    return slope, my - slope * mt


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="静止航向漂移测量",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--duration", type=float, default=120.0, help="采样时长（秒）")
    args = parser.parse_args(argv if argv is not None else sys.argv[1:])

    rclpy.init()
    node = ImuDrift()
    print("开始测量，请保持小车静止、不要碰它。")
    print("采样 {:.0f} 秒 ...".format(args.duration))

    end = time.monotonic() + args.duration
    last_report = time.monotonic()
    try:
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
            now = time.monotonic()
            if now - last_report >= 20.0:
                done = int(now - (end - args.duration))
                msg = "  {}s".format(done)
                for key, data in node.series.items():
                    msg += "  {}={}帧".format(key, len(data))
                print(msg)
                last_report = now
    except KeyboardInterrupt:
        print("提前结束")

    print("")
    print("=" * 62)
    print(" 静止航向漂移报告（采样 {:.0f} s）".format(args.duration))
    print("=" * 62)
    print("  {:<12} {:>8} {:>12} {:>12} {:>10}".format(
        "来源", "帧数", "起止变化", "漂移率", "判断"))

    results = {}
    for key in ("odom", "odom_raw", "imu_data", "yaw_deg"):
        data = node.series[key]
        if len(data) < 20:
            print("  {:<12} {:>8}   没有数据或数据太少".format(key, len(data)))
            continue
        t = [d[0] for d in data]
        raw = [d[1] for d in data]
        y = unwrap_deg(raw)
        slope, _ = linear_slope(t, y)
        rate = slope * 60.0                       # 度/分钟
        span = y[-1] - y[0]
        if abs(rate) < 0.5:
            verdict = "良好"
        elif abs(rate) < 2.0:
            verdict = "偏大"
        else:
            verdict = "必须修"
        results[key] = rate
        print("  {:<12} {:>8} {:>+11.2f}° {:>+9.2f}°/min {:>10}".format(
            key, len(data), span, rate, verdict))

    print("")

    # ---- 里程计模型误差（行驶时最有价值的一段）----
    if "odom_raw" in results and "imu_data" in results:
        raw_rate = results["odom_raw"]
        imu_rate = results["imu_data"]
        diff = raw_rate - imu_rate
        print("  【里程计模型误差】")
        print("    /odom_raw（转向角模型）: {:+.2f}°/min".format(raw_rate))
        print("    /imu/data （真值参考）  : {:+.2f}°/min".format(imu_rate))
        print("    两者之差               : {:+.2f}°/min".format(diff))
        print("")
        if abs(imu_rate) < 0.5 and abs(diff) > 1.0:
            print("    ⚠️ IMU 说基本没转（车在走直），但里程计报出明显角速度 ——")
            print("       base_node_R2 的航向模型（只有转向角，不看左右轮速差）")
            print("       没反映车的真实运动。这就是地图被拧的根源。")
            print("       修法：给 v 乘系数补偿 -> r2_odom 的 yaw_bias_per_m")
        elif abs(diff) < 0.5:
            print("    ✅ 两者接近，里程计模型和实际运动一致。")
        print("")

    if "odom" in results:
        rate = results["odom"]
        print("  换算：以 {:.2f}°/min 建图 20 分钟，地图末端累计偏 {}".format(
            rate, abs(rate) * 20.0))
        if abs(rate) >= 0.5:
            print("")
            print("  ⚠️ /odom 的 yaw 在漂。EKF 的 yaw 来自 /imu/data（madgwick），")
            print("     而 madgwick 的 use_mag=false —— yaw 没有绝对参考，只能靠")
            print("     陀螺仪积分，必然漂。三条修法：")
            print("       1. 打开磁力计：imu_filter_param.yaml 里 use_mag: true")
            print("       2. 换用板载融合：EKF 的 imu0 指向 /imu/yaw_deg 对应的 Imu 源")
            print("       3. 降低 SLAM 对里程计旋转的信任（gmapping 的 str/stt 调大）")
    print("=" * 62)

    try:
        node.destroy_node()
    except Exception:
        pass
    if rclpy.ok():
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""R2 建图前置条件自检（Gate A 的自动化部分）。

检查内容：
  1. 话题频率：/scan、/odom_raw、/imu/data_raw（可自定义）
  2. 消息年龄：now - header.stamp
  3. 时间戳互差：三个传感器的最新帧之间的最大差值
  4. TF 链路：odom->base、base->laser、base->imu，
     以及（--expect-map 时）map->odom、map->laser
  5. 里程计连续性：相邻帧跳变、前进时 x 是否单调
  6. IMU yaw 跳变：区分 ±180° 环绕（已知问题）和真实跳变

用法：
    ros2 run r2_mapping_tools health_check
    ros2 run r2_mapping_tools health_check --duration 15 --expect-map
    ros2 run r2_mapping_tools health_check --json

退出码：0 = 通过（可能有警告），1 = 存在失败项。
"""

import argparse
import json
import math
import sys
import time
from collections import deque

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Float64

import tf2_ros


TOPIC_TYPES = {
    "/scan": (LaserScan, "header"),
    "/scan_filtered": (LaserScan, "header"),
    "/odom_raw": (Odometry, "header"),
    "/odom": (Odometry, "header"),
    "/imu/data_raw": (Imu, "header"),
    "/imu/yaw_deg": (Float64, "none"),
    "/vel_raw": (Twist, "none"),
}

# /imu/yaw_deg 单独列为「可选」：它的类型可能是 std_msgs/Float64 也可能是
# Float32，类型对不上时节点收不到消息，这种情况只报警告不算失败。
DEFAULT_TOPICS = [
    "/scan:10.0",
    "/odom_raw:10.0",
    "/imu/data_raw:10.0",
    "/imu/yaw_deg:10.0",
]
DEFAULT_OPTIONAL_TOPICS = ["/imu/yaw_deg"]

TF_CHECKS = [
    {
        "name": "odom -> base_link",
        "pairs": [("odom", "base_footprint"), ("odom", "base_link")],
        "critical": True,
        "expect_map": False,
    },
    {
        "name": "base_link -> laser",
        "pairs": [("base_link", "laser"), ("base_footprint", "laser")],
        "critical": True,
        "expect_map": False,
    },
    {
        "name": "base_link -> imu_link",
        "pairs": [("base_link", "imu_link"), ("base_footprint", "imu_link")],
        "critical": False,
        "expect_map": False,
    },
    {
        "name": "map -> odom",
        "pairs": [("map", "odom")],
        "critical": True,
        "expect_map": True,
    },
    {
        "name": "map -> laser（全链路）",
        "pairs": [("map", "laser")],
        "critical": True,
        "expect_map": True,
    },
]


def _parse_topic_specs(specs):
    parsed = []
    for spec in specs:
        for item in str(spec).split(","):
            item = item.strip()
            if not item:
                continue
            name, _, hz = item.partition(":")
            try:
                expected = float(hz) if hz else 10.0
            except ValueError:
                expected = 10.0
            parsed.append((name, expected))
    return parsed


class HealthCheck(Node):
    def __init__(self, args):
        super().__init__("r2_mapping_health_check")
        self.args = args

        self.declare_parameter("duration", args.duration)
        self.declare_parameter("topics", args.topics)
        self.declare_parameter("expect_map", args.expect_map)
        self.declare_parameter("warn_skew_ms", args.warn_skew_ms)
        self.declare_parameter("fail_skew_ms", args.fail_skew_ms)

        # 允许用 --ros-args -p <name>:=<value> 覆盖命令行参数
        args.duration = float(self.get_parameter("duration").value)
        args.topics = list(self.get_parameter("topics").value)
        args.expect_map = bool(self.get_parameter("expect_map").value)
        args.warn_skew_ms = float(self.get_parameter("warn_skew_ms").value)
        args.fail_skew_ms = float(self.get_parameter("fail_skew_ms").value)

        self.topic_specs = _parse_topic_specs(args.topics)
        self.optional_topics = set(args.optional_topics)
        self.samples = {}          # topic -> deque[(arrival_monotonic, stamp_sec)]
        self.unsupported = []

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=50,
        )

        for name, _ in self.topic_specs:
            entry = TOPIC_TYPES.get(name)
            if entry is None:
                self.unsupported.append(name)
                continue
            msg_type, stamp_kind = entry
            self.samples[name] = deque(maxlen=4000)
            self._make_sub(name, msg_type, stamp_kind, qos)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self._tf_start = None
        self.tf_stats = {
            check["name"]: {"ok": 0, "total": 0, "age": None, "static": False}
            for check in TF_CHECKS
        }

        self.odom_jumps = []
        self.odom_dir_suspect = 0
        self.odom_travel = 0.0
        self._last_odom = None

        self.yaw_wraps = 0
        self.yaw_real_jumps = 0
        self.yaw_first = None
        self.yaw_last = None
        self._last_yaw = None

    # ---------------------------------------------------------------- 订阅
    def _make_sub(self, name, msg_type, stamp_kind, qos):
        def callback(msg, _name=name, _kind=stamp_kind):
            self._record(_name, _kind, msg)

        self.create_subscription(msg_type, name, callback, qos)

    def _stamp_of(self, msg, kind):
        if kind == "header" and hasattr(msg, "header"):
            return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        return None

    def _record(self, name, kind, msg):
        now = time.monotonic()
        stamp = self._stamp_of(msg, kind)
        self.samples[name].append((now, stamp))

        if name == "/odom_raw":
            self._on_odom(msg, stamp)
        elif name == "/imu/yaw_deg":
            self._on_yaw(msg, now)

    def _on_odom(self, msg, stamp):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        vx = msg.twist.twist.linear.x
        if self._last_odom is not None:
            lx, ly, lt, _ = self._last_odom
            dt = stamp - lt if stamp is not None else 0.0
            dx = x - lx
            dy = y - ly
            dist = math.hypot(dx, dy)
            if 0.0 < dt < 1.0:
                self.odom_travel += dist
                if dist > self.args.odom_jump_m:
                    self.odom_jumps.append(dist)
                # 前进指令下位置却在后退 -> 方向可疑
                if vx > 0.05 and dx < -0.01:
                    self.odom_dir_suspect += 1
                if vx < -0.05 and dx > 0.01:
                    self.odom_dir_suspect += 1
        self._last_odom = (x, y, stamp, vx)

    def _on_yaw(self, msg, now):
        value = float(msg.data)
        self.yaw_last = value
        if self.yaw_first is None:
            self.yaw_first = value
        if self._last_yaw is not None:
            _, last_value = self._last_yaw
            diff = value - last_value
            if diff > 180.0:
                diff -= 360.0
            elif diff < -180.0:
                diff += 360.0
            raw = value - last_value
            if abs(abs(raw) - 360.0) < 20.0:
                self.yaw_wraps += 1
            elif abs(diff) > self.args.yaw_jump_deg:
                self.yaw_real_jumps += 1
        self._last_yaw = (now, value)

    # ------------------------------------------------------------------ TF
    def _check_tf(self):
        now = self.get_clock().now()
        if self._tf_start is None:
            # TF 缓冲区要先填数据，头两秒的查询失败不算问题
            self._tf_start = time.monotonic()
            return
        if time.monotonic() - self._tf_start < 2.0:
            return
        for check in TF_CHECKS:
            if check["expect_map"] and not self.args.expect_map:
                continue
            stats = self.tf_stats[check["name"]]
            stats["total"] += 1
            age = None
            for parent, child in check["pairs"]:
                try:
                    tf = self.tf_buffer.lookup_transform(
                        parent, child, Time(), Duration(seconds=0.05)
                    )
                except Exception:
                    continue
                if tf.header.stamp.sec == 0 and tf.header.stamp.nanosec == 0:
                    # 静态 TF 的时间戳恒为 0，算"年龄"没有意义
                    age = 0.0
                    stats["static"] = True
                else:
                    stamp = Time.from_msg(tf.header.stamp)
                    age = (now - stamp).nanoseconds / 1e9
                break
            if age is not None:
                stats["ok"] += 1
                stats["age"] = age

    # --------------------------------------------------------------- 报告
    def _topic_report(self):
        rows = []
        stamped_latest = {}

        for name, expected in self.topic_specs:
            if name in self.unsupported:
                rows.append(
                    {
                        "topic": name,
                        "status": "SKIP",
                        "hz": None,
                        "expected_hz": expected,
                        "age_s": None,
                        "note": "未知消息类型，未监控",
                    }
                )
                continue

            samples = self.samples.get(name, ())
            if len(samples) < 2:
                optional = name in self.optional_topics
                rows.append(
                    {
                        "topic": name,
                        "status": "WARN" if optional else "FAIL",
                        "hz": 0.0,
                        "expected_hz": expected,
                        "age_s": None,
                        "note": (
                            "没有数据（可选话题；若确实存在，检查消息类型是否 "
                            "std_msgs/Float64）"
                            if optional
                            else "没有数据（{} 帧）".format(len(samples))
                        ),
                    }
                )
                continue

            t_first = samples[0][0]
            t_last = samples[-1][0]
            span = max(1e-6, t_last - t_first)
            hz = (len(samples) - 1) / span

            stamp = samples[-1][1]
            age = None
            if stamp is not None and stamp > 0.0:
                now_msg = self.get_clock().now().to_msg()
                age = (now_msg.sec + now_msg.nanosec * 1e-9) - stamp
                stamped_latest[name] = stamp

            if hz < 0.5 * expected:
                status = "FAIL"
            elif hz < 0.8 * expected or hz > 1.5 * expected:
                status = "WARN"
            else:
                status = "PASS"
            if age is not None and age > 1.0:
                status = "FAIL"

            rows.append(
                {
                    "topic": name,
                    "status": status,
                    "hz": hz,
                    "expected_hz": expected,
                    "age_s": age,
                    "note": "{} 帧".format(len(samples)),
                }
            )

        return rows, stamped_latest

    def _skew_ms(self, stamped_latest):
        # 只用传感器类话题算互差，避免把 /imu/yaw_deg 这种无时间戳的算进来
        values = [
            v
            for k, v in stamped_latest.items()
            if k in ("/scan", "/scan_filtered", "/odom_raw", "/imu/data_raw")
        ]
        if len(values) < 2:
            return None
        return (max(values) - min(values)) * 1000.0

    def report(self):
        topic_rows, stamped_latest = self._topic_report()
        skew_ms = self._skew_ms(stamped_latest)

        failures = []
        warnings = []

        for row in topic_rows:
            if row["status"] == "FAIL":
                failures.append(
                    "话题 {} 异常：{}".format(row["topic"], row["note"])
                )
            elif row["status"] == "WARN":
                warnings.append("话题 {} 频率偏离".format(row["topic"]))

        tf_rows = []
        for check in TF_CHECKS:
            if check["expect_map"] and not self.args.expect_map:
                continue
            stats = self.tf_stats[check["name"]]
            if stats["total"] == 0:
                continue
            ratio = stats["ok"] / stats["total"]
            if ratio < 0.5:
                status = "FAIL"
            elif ratio < 0.95:
                status = "WARN"
            else:
                status = "PASS"
            if status == "FAIL" and check["critical"]:
                failures.append("TF 缺失：{}".format(check["name"]))
            elif status == "FAIL":
                warnings.append("TF 不可用（非关键）：{}".format(check["name"]))
            elif status == "WARN":
                warnings.append("TF 时断时续：{}".format(check["name"]))
            tf_rows.append(
                {
                    "name": check["name"],
                    "status": status,
                    "ok_ratio": ratio,
                    "age_s": stats["age"],
                    "static": stats.get("static", False),
                    "critical": check["critical"],
                }
            )

        if skew_ms is not None:
            if skew_ms > self.args.fail_skew_ms:
                failures.append("时间戳互差距过大：{:.0f} ms".format(skew_ms))
            elif skew_ms > self.args.warn_skew_ms:
                warnings.append("时间戳互差偏大：{:.0f} ms".format(skew_ms))

        if self.odom_jumps:
            warnings.append(
                "里程计跳变 {} 次，最大 {:.3f} m".format(
                    len(self.odom_jumps), max(self.odom_jumps)
                )
            )
        if self.odom_dir_suspect > 0:
            warnings.append(
                "里程计方向可疑 {} 次（前进指令下 x 反向）".format(
                    self.odom_dir_suspect
                )
            )
        if self.yaw_real_jumps > 0:
            failures.append("IMU yaw 真实跳变 {} 次".format(self.yaw_real_jumps))
        if self.yaw_wraps > 0:
            warnings.append(
                "IMU yaw 跨 ±180° 环绕 {} 次（已知问题，下游需 unwrap）".format(
                    self.yaw_wraps
                )
            )

        result = {
            "ok": not failures,
            "failures": failures,
            "warnings": warnings,
            "topics": topic_rows,
            "tf": tf_rows,
            "stamp_skew_ms": skew_ms,
            "odom": {
                "travel_m": self.odom_travel,
                "jumps": len(self.odom_jumps),
                "max_jump_m": max(self.odom_jumps) if self.odom_jumps else 0.0,
                "direction_suspect": self.odom_dir_suspect,
            },
            "imu_yaw": {
                "wraps": self.yaw_wraps,
                "real_jumps": self.yaw_real_jumps,
                "start_deg": self.yaw_first,
                "end_deg": self.yaw_last,
            },
        }
        return result


def _print_report(result, duration):
    print("")
    print("=" * 66)
    print(" R2 建图前置条件自检（采样 {:.0f} s）".format(duration))
    print("=" * 66)

    print("\n[话题]")
    print("  {:<18} {:>8} {:>8} {:>9}  {:<6} {}".format(
        "话题", "频率Hz", "期望", "年龄s", "结论", "说明"))
    for row in result["topics"]:
        hz = "{:.1f}".format(row["hz"]) if row["hz"] is not None else "-"
        age = "{:.2f}".format(row["age_s"]) if row["age_s"] is not None else "-"
        print("  {:<18} {:>8} {:>8.1f} {:>9}  {:<6} {}".format(
            row["topic"], hz, row["expected_hz"], age, row["status"], row["note"]))

    print("\n[TF]")
    for row in result["tf"]:
        if row.get("static"):
            age = "静态"
        else:
            age = "{:.2f} s".format(row["age_s"]) if row["age_s"] is not None else "-"
        print("  {:<26} {:>6}  可用率 {:>5.0f}%  延迟 {}{}".format(
            row["name"], row["status"], row["ok_ratio"] * 100.0, age,
            "" if row["critical"] else "  (非关键)"))

    skew = result["stamp_skew_ms"]
    print("\n[时间戳]")
    print("  传感器最新帧互差：{}".format(
        "{:.0f} ms".format(skew) if skew is not None else "无法计算（缺少两个以上带时间戳的话题）"))

    odom = result["odom"]
    print("\n[里程计 /odom_raw]")
    print("  采样位移 {:.3f} m，跳变 {} 次（最大 {:.3f} m），方向可疑 {} 次".format(
        odom["travel_m"], odom["jumps"], odom["max_jump_m"],
        odom["direction_suspect"]))
    print("  注意：方向与标尺需要人工确认（直行 1 m 对比卷尺）")

    yaw = result["imu_yaw"]
    if yaw["start_deg"] is not None:
        print("\n[IMU yaw]")
        print("  起始 {:.1f}° → 结束 {:.1f}°，环绕 {} 次，真实跳变 {} 次".format(
            yaw["start_deg"], yaw["end_deg"], yaw["wraps"], yaw["real_jumps"]))

    print("")
    print("-" * 66)
    if result["warnings"]:
        print("警告 {} 项：".format(len(result["warnings"])))
        for item in result["warnings"]:
            print("  ! " + item)
    if result["failures"]:
        print("失败 {} 项：".format(len(result["failures"])))
        for item in result["failures"]:
            print("  x " + item)
        print("\n结论：前置条件未满足，先解决上面的失败项再建图。")
    else:
        print("结论：前置条件通过（{} 项警告）。".format(len(result["warnings"])))
        print("人工补充确认：里程计方向/标尺、LiDAR 外参、地图闭环。")
    print("-" * 66)


def _publish_health(node, result):
    """如果 r2_mapping_msgs 已编译，则顺带把结果发成消息（可选）。"""
    try:
        from r2_mapping_msgs.msg import MappingHealth, TopicHealth
    except Exception:
        return

    publisher = node.create_publisher(MappingHealth, "/r2_mapping/health", 5)
    msg = MappingHealth()
    msg.stamp = node.get_clock().now().to_msg()
    msg.ok = bool(result["ok"])
    msg.max_stamp_skew_s = float(result["stamp_skew_ms"] or 0.0) / 1000.0
    msg.tf_tree_complete = all(row["status"] == "PASS" for row in result["tf"])
    msg.warnings = list(result["warnings"]) + list(result["failures"])

    for row in result["topics"]:
        item = TopicHealth()
        item.topic = row["topic"]
        item.measured_hz = float(row["hz"] or 0.0)
        item.expected_hz = float(row["expected_hz"])
        item.age_s = float(row["age_s"] or 0.0)
        item.ok = row["status"] != "FAIL"
        item.note = str(row["note"])
        msg.topics.append(item)

    publisher.publish(msg)


def _build_parser():
    parser = argparse.ArgumentParser(
        description="R2 建图前置条件自检",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--duration", type=float, default=10.0,
                        help="采样时长（秒）")
    parser.add_argument("--topics", nargs="+", default=DEFAULT_TOPICS,
                        help='要检查的话题，形如 "/scan:10.0"')
    parser.add_argument("--optional-topics", nargs="+", default=DEFAULT_OPTIONAL_TOPICS,
                        help="允许缺失的话题（只警告不判失败）")
    parser.add_argument("--expect-map", action="store_true",
                        help="定位复测模式：额外检查 map->odom、map->laser")
    # 三个传感器各自独立 10 Hz 采样时，最新帧的互差天然就会在 0~2 个周期
    # （0~200 ms）之间波动，再加上各自的发布延迟，所以阈值必须按"周期"定，
    # 不能拍一个 50 ms 出来，否则每次都误报。
    # 真正的时钟错位（某个节点用了 wall time / sim time）通常是秒级，抓得住。
    parser.add_argument("--warn-skew-ms", type=float, default=200.0,
                        help="约 2 个采样周期")
    parser.add_argument("--fail-skew-ms", type=float, default=500.0,
                        help="约 5 个采样周期，通常是时钟错位")
    parser.add_argument("--odom-jump-m", type=float, default=0.5)
    parser.add_argument("--yaw-jump-deg", type=float, default=30.0)
    parser.add_argument("--json", action="store_true", help="额外输出 JSON")
    return parser


def main(argv=None):
    args = _build_parser().parse_args(argv if argv is not None else sys.argv[1:])

    rclpy.init()
    node = HealthCheck(args)
    try:
        duration = node.args.duration
        node.get_logger().info("开始采样 {:.0f} s ...".format(duration))
        deadline = time.monotonic() + max(1.0, duration)
        next_tf = 0.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.monotonic()
            if now >= next_tf:
                node._check_tf()
                next_tf = now + 0.5

        result = node.report()
        _publish_health(node, result)
        _print_report(result, duration)
        if args.json:
            print(json.dumps(result, indent=2, ensure_ascii=False))
        return 0 if result["ok"] else 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())

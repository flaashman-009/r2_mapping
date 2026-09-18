#!/usr/bin/env python3
"""前轮回正 / 转向指令验证工具。

背景：这台车的底盘固件疑似把 `cmd_vel.linear.y == 0` 当成"不更新转向"，
导致松杆后前轮停在原地不回正。本工具用来验证这一点并完成回正。

模式：
    # 只读当前转向反馈（单位 deg，来自 /vel_raw.linear.y）
    python3 steer_center.py --read

    # 保持某个转向值（默认 2 秒），再读反馈
    python3 steer_center.py --hold 0.01

    # 完整探针：0 -> +0.01 -> 0，判断固件认不认 0
    python3 steer_center.py --probe

    # 回正（发一个极小非零值）
    python3 steer_center.py --center

单位的约定：`linear.y = 转向角(deg) / 1000`，所以 0.03 = 30°。
"""

import argparse
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

CMD_TOPIC = "/cmd_vel"
FEEDBACK_TOPIC = "/vel_raw"


class SteerTool(Node):

    def __init__(self):
        super().__init__("steer_center_tool")
        self.pub = self.create_publisher(Twist, CMD_TOPIC, 10)
        self.latest = None
        self.sub = self.create_subscription(
            Twist, FEEDBACK_TOPIC, self._on_feedback, 10
        )
        self.get_logger().info(
            "发布 {}，订阅 {}".format(CMD_TOPIC, FEEDBACK_TOPIC)
        )

    def _on_feedback(self, msg):
        self.latest = msg.linear.y

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def read_steer(self, timeout=6.0):
        """等待并读回当前转向反馈（deg）。

        刚启动时 DDS 发现需要时间，所以这里等到真的收到数据再返回。
        """
        self.latest = None
        end = time.monotonic() + timeout
        first_at = None
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.latest is not None:
                if first_at is None:
                    first_at = time.monotonic()
                elif time.monotonic() - first_at > 0.3:
                    break
        return self.latest

    def hold(self, y, seconds, rate=20.0):
        """按指定频率持续发布转向指令。"""
        period = 1.0 / rate
        end = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < end:
            twist = Twist()
            twist.linear.y = float(y)
            self.pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=period)

    def read_cmd_echo(self, y, seconds=1.5):
        """发 y 一段时间，然后读反馈。"""
        self.hold(y, seconds)
        return self.read_steer()


def _fmt(value):
    return "None" if value is None else "{:+.2f} deg".format(value)


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="前轮回正 / 转向指令验证",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--read", action="store_true", help="只读当前转向反馈")
    parser.add_argument("--hold", type=float, default=None,
                        help="持续发布这个 linear.y（deg/1000）")
    parser.add_argument("--seconds", type=float, default=2.0, help="保持时长")
    parser.add_argument("--probe", action="store_true",
                        help="探针：0 -> +0.01 -> 0，判断固件认不认 0")
    parser.add_argument("--center", action="store_true",
                        help="回正：发一个极小非零值")
    parser.add_argument("--epsilon", type=float, default=0.001,
                        help="回正用的极小非零值（deg/1000，0.001 = 1°）")
    args = parser.parse_args(argv if argv is not None else sys.argv[1:])

    rclpy.init()
    node = SteerTool()
    try:
        node.spin_for(1.0)

        if args.read:
            print("当前转向反馈：{}".format(_fmt(node.read_steer())))
            return 0

        if args.hold is not None:
            print("发 linear.y = {:.4f}（{:.1f}°）保持 {:.1f} s".format(
                args.hold, args.hold * 1000.0, args.seconds))
            before = node.read_steer()
            after = node.read_cmd_echo(args.hold, args.seconds)
            print("  发送前：{}".format(_fmt(before)))
            print("  发送后：{}".format(_fmt(after)))
            return 0

        if args.center:
            before = node.read_steer()
            print("回正前：{}".format(_fmt(before)))
            print("发 linear.y = {:.4f}（{:.1f}°）".format(
                args.epsilon, args.epsilon * 1000.0))
            after = node.read_cmd_echo(args.epsilon, 2.0)
            print("回正后：{}".format(_fmt(after)))
            node.spin_for(1.5)
            settled = node.read_steer()
            print("再等 1.5 s（看会不会被别的节点覆盖）：{}".format(_fmt(settled)))
            return 0

        if args.probe:
            print("=" * 56)
            print(" 转向指令探针")
            print("=" * 56)
            baseline = node.read_steer()
            print("1. 当前反馈（基线）        : {}".format(_fmt(baseline)))

            v1 = node.read_cmd_echo(0.01, 2.0)
            print("2. 发 +0.01（+10°）后       : {}".format(_fmt(v1)))

            v2 = node.read_cmd_echo(0.0, 2.0)
            print("3. 发 0.0 之后              : {}".format(_fmt(v2)))

            node.spin_for(1.0)
            v3 = node.read_steer()
            print("4. 再等 1 s                 : {}".format(_fmt(v3)))

            print("")
            if v1 is not None and abs(v1) > 5.0:
                print("→ +0.01 生效了，转向指令通路正常。")
            elif v1 is not None:
                print("→ +0.01 没生效，可能有别的节点在覆盖 /cmd_vel。")

            if v2 is not None and v3 is not None:
                if abs(v2) > 5.0 or abs(v3) > 5.0:
                    print("→ 发 0.0 之后角度没有回到 0：**固件确实忽略 0**，")
                    print("  必须用非零小值回正（steer_epsilon 的做法是对的）。")
                else:
                    print("→ 发 0.0 之后角度回到 0：固件认 0，")
                    print("  那前轮没回正的原因在别处（可能是伺服没通电）。")
            return 0

        parser.print_help()
        return 0
    finally:
        try:
            if rclpy.ok():
                node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())

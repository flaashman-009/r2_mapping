#!/usr/bin/env python3
"""把 Nav2 的 (v, ω) 转成 R2 底盘原生的转向指令。

## 为什么需要它（2026-09-16 实测证据）

这台车的底盘有两条转向通道：

    linear.y   —— 直接给转向角，值 × 1000 = 度数
                  实测 0.02 -> 20.0°，-0.02 -> -20.0°，任何车速都立即生效
    angular.z  —— 固件自己按车速换算，但有两个坑：
                  (a) **车静止时不生效**：v=0 时发 0.5 或 -0.5，前轮都停在 0°
                  (b) **行驶中发 0 时"保持上一次角度"**：实测运动 567 帧里
                      有 67 帧 angular.z=0，那 67 帧前轮全部卡在 -25°

坑 (b) 是致命的：Nav2 的语义是"ω=0 = 直行"，底盘却理解成"不更新转向"。
于是车在前轮打着 25° 的情况下"直行"——实际在画弧，控制器却以为在走直线。
点云因此持续左偏右偏，误差一路累积，最后 AMCL 迷失。

## 本节点的做法

不依赖固件的换算，自己按阿克曼模型算，然后走 **linear.y** 这条直接通道：

    δ = atan(ω · L / v)          # L = 轴距 0.2681 m
    δ 限幅到 ±max_steer_deg      # 默认 40°，机械极限 44°，留余量
    再做转速率限制                # 默认 70°/s，让舵机平滑，治"暖胎"

输出：

    linear.x   = v                （原样透传）
    linear.y   = δ_deg / 1000     （底盘认这个）
    angular.z  = 0

**δ 接近 0 时绝不发 0**：固件把 0 当"不更新"，所以强制发 ±eps（默认 1°），
这和 scripts/steer_center_cmd.sh 回正是同一个道理。

## 用法

    python3 cmd_vel_ackermann.py --ros-args -p wheelbase:=0.2681

Nav2 那边要把 controller_server 和 behavior_server 的输出 remap 到
/cmd_vel_nav，本节点订阅 /cmd_vel_nav、发布 /cmd_vel（驱动听的那个）。
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool


class CmdVelAckermann(Node):

    def __init__(self):
        super().__init__("cmd_vel_ackermann")

        self.declare_parameter("wheelbase", 0.2681)
        self.declare_parameter("max_steer_deg", 40.0)
        # 倒车时的转向限幅（更严）。
        # 原因：δ = atan(ω·L/|v|)，倒车时 |v| 往往很小（0.05~0.20），
        # 这个比值会被放大 —— 实测倒车时转向打到 17~35°，车在小空间里
        # 大幅摆动，扫描剧烈变化，是定位崩溃的常见触发点。
        self.declare_parameter("reverse_steer_deg", 22.0)
        # 低速时的转向限幅（见下面 _tick 里的说明）。
        # 为什么需要：δ=atan(ω·L/v)，TEB 的 ω 不随车速减小，
        # 车一减速 δ 就被放大 → 到达终点前的前轮猛打 → 车急转。
        self.declare_parameter("low_speed", 0.12)           # m/s
        self.declare_parameter("low_speed_steer_deg", 20.0)
        self.declare_parameter("max_steer_rate_dps", 45.0)
        self.declare_parameter("steer_deadband_deg", 1.5)
        self.declare_parameter("min_speed", 0.05)
        self.declare_parameter("eps_deg", 1.0)
        self.declare_parameter("steer_trim_deg", 0.0)
        self.declare_parameter("input_topic", "/cmd_vel_nav")
        self.declare_parameter("output_topic", "/cmd_vel")
        self.declare_parameter("halt_topic", "/cmd_vel_halt")
        self.declare_parameter("cmd_timeout", 0.5)
        self.declare_parameter("publish_rate", 20.0)
        self.declare_parameter("log_every", 0)   # >0 时每 N 条指令打一行日志

        self.L = self.get_parameter("wheelbase").value
        self.max_steer = self.get_parameter("max_steer_deg").value
        self.reverse_steer = self.get_parameter("reverse_steer_deg").value
        self.low_speed = self.get_parameter("low_speed").value
        self.low_speed_steer = self.get_parameter("low_speed_steer_deg").value
        self.max_rate = self.get_parameter("max_steer_rate_dps").value
        self.deadband = self.get_parameter("steer_deadband_deg").value
        self.min_speed = self.get_parameter("min_speed").value
        self.eps = self.get_parameter("eps_deg").value
        self.trim = self.get_parameter("steer_trim_deg").value
        in_topic = self.get_parameter("input_topic").value
        out_topic = self.get_parameter("output_topic").value
        halt_topic = self.get_parameter("halt_topic").value
        self.timeout = self.get_parameter("cmd_timeout").value
        rate = self.get_parameter("publish_rate").value
        self.log_every = int(self.get_parameter("log_every").value)

        self.v = 0.0            # 期望车速
        self.delta_cmd = 0.0    # 期望转向角（度，不含 trim）
        self.delta_out = 0.0    # 实际发出去的转向角（度，含 trim，已限速率）
        self.last_cmd_t = None
        self.last_pub_t = self.get_clock().now()
        self.n_msg = 0
        self.n_clip = 0
        self.max_abs_delta = 0.0
        self.n_stale_tick = 0
        self.n_deadband_skip = 0
        self.halted = False

        self.create_subscription(Twist, in_topic, self._on_cmd, 10)
        # 迷失看门狗通过这个话题叫停（True = 立即停车）
        self.create_subscription(Bool, halt_topic, self._on_halt, 10)
        self.pub = self.create_publisher(Twist, out_topic, 10)
        self.create_timer(1.0 / max(1.0, rate), self._tick)

        self.get_logger().info(
            "cmd_vel_ackermann 启动：{} -> {} | 轴距 {:.4f} m | "
            "转向限幅 ±{:.0f}°（倒车 ±{:.0f}°）| 转速率上限 {:.0f}°/s | "
            "死区 {:.1f}° | 最小速度 {:.2f} m/s".format(
                in_topic, out_topic, self.L, self.max_steer, self.reverse_steer,
                self.max_rate, self.deadband, self.min_speed))
        self.get_logger().info(
            "注意：输出走 linear.y（值×1000=度），angular.z 恒为 0。"
            "δ≈0 时强制发 ±{:.0f}° 以防止固件'保持上次角度'。".format(self.eps))

    # ------------------------------------------------------------------
    def _on_cmd(self, msg):
        self.n_msg += 1
        self.last_cmd_t = self.get_clock().now()

        v = msg.linear.x
        w = msg.angular.z          # Nav2 给的是角速度 rad/s
        self.v = v

        if abs(v) < self.min_speed:
            # 车速太低时 atan(ωL/v) 会爆掉，也没意义（阿克曼不能原地转）
            self.delta_cmd = 0.0 if abs(w) < 1e-3 else self.delta_out
        else:
            self.delta_cmd = math.degrees(math.atan(w * self.L / abs(v)))
            if v < 0:                      # 倒车时转向符号要反过来
                self.delta_cmd = -self.delta_cmd

        if self.log_every > 0 and self.n_msg % self.log_every == 1:
            self.get_logger().info(
                "收到 #{}: v={:+.3f} w={:+.3f} -> delta={:+.1f} deg".format(
                    self.n_msg, v, w, self.delta_cmd))

    def _on_halt(self, msg):
        """迷失看门狗叫停：True = 立即停车并回正。"""
        new = bool(msg.data)
        if new != self.halted:
            self.halted = new
            if new:
                self.get_logger().error("收到停车指令（看门狗）：立即停车")
            else:
                self.get_logger().warn("停车指令解除，恢复接受导航指令")

    def _tick(self):
        now = self.get_clock().now()

        # 超时看门狗：没有新指令就停车
        stale = (self.last_cmd_t is None or
                 (now - self.last_cmd_t).nanoseconds * 1e-9 > self.timeout)
        if stale:
            self.n_stale_tick += 1
            if self.log_every > 0 and self.n_stale_tick % 40 == 1:
                self.get_logger().warn(
                    "看门狗复位：第 {} 个空转 tick 没有新指令".format(
                        self.n_stale_tick))
        stop = stale or self.halted
        v_out = 0.0 if stop else self.v
        target = 0.0 if stop else self.delta_cmd

        # ---- 转向限幅（分三种情况）----
        # 2026-09-19 加"低速限幅"，这是修"到终点就飘"的关键一刀。
        #
        # 实测：到达终点前必然减速，而 TEB 的 ω 指令不会跟着减小，
        # 于是 δ = atan(ω·L/v) 被放大：
        #     v=0.23 → 19°    v=0.15 → 28°    v=0.07 → 49°（被限到 40°）
        # 实测停车前 2 秒：v 从 0.23 掉到 0.07，前轮从 −1° 猛打到 +39°，
        # 车在 2 秒内急转 51°（IMU 实测），AMCL 跟不上 → 残差 0.35→0.64
        # → 表现就是"每次都在终点飘"。
        #
        # 低速大转角除了造成急转没有任何好处：0.1 m/s 下 20° 转向
        # 对应 0.74 m 转弯半径，完全够用。所以让限幅随速度平滑变化：
        #     v=0        → ±low_speed_steer（默认 20°）
        #     v≥low_speed → ±max_steer（默认 40°）
        v_abs = abs(v_out)
        if v_out < -1e-6:
            limit = self.reverse_steer                      # 倒车：最严
        elif v_abs >= self.low_speed:
            limit = self.max_steer                          # 正常行驶
        else:
            f = v_abs / max(1e-6, self.low_speed)           # 0~1 平滑过渡
            limit = self.low_speed_steer + \
                (self.max_steer - self.low_speed_steer) * f
        clipped = max(-limit, min(limit, target))
        if abs(clipped - target) > 1e-6:
            self.n_clip += 1

        # 转速率限制 + 死区：
        #   死区：目标变化小于 deadband 就不动 —— 抹掉控制器的微抖动，
        #         代价是最多 lag 一个 deadband 的角度（1.5° 在 0.23 m/s 下
        #         只对应 10 m 转弯半径，可以忽略）。
        #   速率：即使变化大，也按 max_rate 度/秒 慢慢转过去。
        dt = max(1e-3, (now - self.last_pub_t).nanoseconds * 1e-9)
        self.last_pub_t = now
        d = clipped - self.delta_out
        if abs(d) < self.deadband:
            self.n_deadband_skip += 1
            d = 0.0
        step = self.max_rate * dt
        if abs(d) > step:
            d = step if d > 0 else -step
        self.delta_out += d
        self.max_abs_delta = max(self.max_abs_delta, abs(self.delta_out))

        # 关键：绝不发 exactly 0，固件会当成"不更新"
        deg = self.delta_out + self.trim
        if abs(deg) < self.eps:
            deg = self.eps if deg >= 0 else -self.eps

        out = Twist()
        out.linear.x = float(v_out)
        out.linear.y = float(deg / 1000.0)   # 底盘：值 × 1000 = 度
        out.linear.z = 0.0
        out.angular.z = 0.0
        self.pub.publish(out)


def main():
    rclpy.init()
    node = CmdVelAckermann()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(
            "统计：收到 {} 条指令，限幅 {} 次，死区跳过 {} 次，"
            "最大转向 {:.1f}°".format(
                node.n_msg, node.n_clip, node.n_deadband_skip,
                node.max_abs_delta))
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

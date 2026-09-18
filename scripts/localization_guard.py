#!/usr/bin/env python3
"""定位护卫：协方差过大就停车，等定位恢复再让你发新目标。

## 它解决的问题

2026-09-18 记录分析（1015 秒）：

  * 最大 4 次位姿瞬移：39.4 / 27.4 / 40.1 / 38.0 米
  * 其中 3 次发生在**倒车指令后 1~2 秒内**
  * 崩溃前的共同前兆：**AMCL 协方差从 0.1 飙到 85 → 196 → 224**
  * 崩完之后，AMCL 在错误位姿上停了 **50 秒**（点云残差 0.6~0.85）
  * 里程计全程平滑 —— 不是里程计跳变，是粒子滤波自己崩了

危险不在于"停车后自己开"，而在于：**位姿已经错了，Nav2 还在按错误的
位姿规划，于是车朝错误方向走**（9-16 那次"反着走"就是航向整体翻转 177°）。

## 它做什么

只订阅 /amcl_pose（代价极低，不用地图、不用激光）：

    协方差 > cov_stop 持续 t_stop 秒
        → 通过 /cmd_vel_halt 让转向适配节点停车（原地停住）
        → 终端红色报警，提示用 RViz 的 2D Pose Estimate 重新给位姿

    协方差 < cov_ok 持续 t_resume 秒
        → 自动解除，可以发新目标

**不需要"取消 goal"**：Nav2 里发新目标会自动抢占旧目标。
所以流程是：停车 → 你确认位姿 → 发新目标。

"确定自身位置"这一步最可靠的是**人工**：在 RViz 里用 2D Pose Estimate
拖到车的真实位置。让 AMCL 自己瞎找（全局重定位）经常找错。

用法：
    python3 localization_guard.py
    python3 localization_guard.py --ros-args -p cov_stop:=20.0 -p t_stop:=2.0
"""

import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool


class LocalizationGuard(Node):

    def __init__(self):
        super().__init__("localization_guard")

        self.declare_parameter("halt_topic", "/cmd_vel_halt")
        self.declare_parameter("cov_stop", 20.0)      # 超过它就认为定位崩了
        self.declare_parameter("t_stop", 2.0)         # 持续这么久才停
        self.declare_parameter("cov_ok", 1.0)         # 低于它才算恢复
        self.declare_parameter("t_resume", 5.0)       # 恢复持续这么久才解除
        self.declare_parameter("arm_cov", 2.0)        # 协方差低于它才算"定位好过"
        self.declare_parameter("auto_resume", True)   # False = 只报警不停车
        self.declare_parameter("check_rate", 5.0)

        halt_topic = self.get_parameter("halt_topic").value
        self.cov_stop = self.get_parameter("cov_stop").value
        self.t_stop = self.get_parameter("t_stop").value
        self.cov_ok = self.get_parameter("cov_ok").value
        self.t_resume = self.get_parameter("t_resume").value
        self.arm_cov = self.get_parameter("arm_cov").value
        self.auto_resume = self.get_parameter("auto_resume").value
        rate = self.get_parameter("check_rate").value

        self.cov = 0.0
        self.n_pose = 0
        self.armed = False
        self.halted = False
        self.bad_since = None
        self.good_since = None
        self.n_stop = 0

        qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose",
                                 self._on_amcl, qos)
        self.pub = self.create_publisher(Bool, halt_topic, 10)
        self.create_timer(1.0 / max(0.5, rate), self._tick)

        self.get_logger().info(
            "定位护卫启动：协方差 > {:.0f} 持续 {:.0f}s 即停车；"
            "恢复到 < {:.1f} 持续 {:.0f}s 自动解除".format(
                self.cov_stop, self.t_stop, self.cov_ok, self.t_resume))

    def _on_amcl(self, msg):
        self.n_pose += 1
        c = msg.pose.covariance
        self.cov = c[0] + c[7] + c[35]

    def _halt(self, on):
        self.halted = on
        m = Bool()
        m.data = on
        self.pub.publish(m)
        if on:
            self.n_stop += 1
            self.get_logger().error(
                "★★★ 定位已崩（协方差 {:.0f} > {:.0f}）→ 已停车".format(
                    self.cov, self.cov_stop))
            self.get_logger().error(
                "     下一步：在 RViz 里用 **2D Pose Estimate** 把车拖到"
                "真实位置，等协方差降下来，再重新发目标。")
            self.get_logger().error(
                "     （不要在当前位姿下继续发目标，否则会朝错误方向走）")
        else:
            self.get_logger().warn(
                "定位已恢复（协方差 {:.2f}），停车解除，可以发新目标".format(
                    self.cov))

    def _tick(self):
        now = time.monotonic()
        if self.n_pose < 20:
            return                      # 还没收到足够的 AMCL 数据

        if self.cov > self.cov_stop:
            self.good_since = None
            if not self.armed:
                return                  # 一开始定位就不好，不算"崩"
            if self.bad_since is None:
                self.bad_since = now
            elif not self.halted and now - self.bad_since >= self.t_stop:
                if self.auto_resume:
                    self._halt(True)
                else:
                    self.get_logger().error(
                        "★ 定位异常（协方差 {:.0f}）——仅报警未停车".format(
                            self.cov))
                    self.bad_since = None
        else:
            self.bad_since = None
            if not self.armed and self.cov < self.arm_cov:
                self.armed = True
                self.get_logger().info(
                    "定位良好（协方差 {:.2f}）→ 护卫已武装".format(self.cov))
            if self.halted:
                if self.cov < self.cov_ok:
                    if self.good_since is None:
                        self.good_since = now
                    elif now - self.good_since >= self.t_resume:
                        self._halt(False)
                else:
                    self.good_since = None


def main():
    rclpy.init()
    node = LocalizationGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("定位护卫统计：触发停车 {} 次".format(node.n_stop))
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

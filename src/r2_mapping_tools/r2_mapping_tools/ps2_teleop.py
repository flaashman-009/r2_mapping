#!/usr/bin/env python3
"""PS2 手柄遥控节点（替代原厂 yahboom_joy_R2）。

为什么不用原厂节点：
  实测这台车的 DragonRise 手柄（0079:181c）**不会产生 buttons[9]**，
  而原厂 `yahboom_joy_R2.py` 把 buttons[9] 硬编码成遥控解锁键，
  结果就是永远解锁不了、/cmd_vel 一个字节都不发。
  另外原厂节点没有任何断流保护，8-29 那次失控就发生在它身上。

本节点的设计目标：
  1. 轴/键索引全部参数化，不依赖厂商假设；
  2. 有明确的解锁开关，且默认**上锁**；
  3. 有看门狗：/joy 断流超过阈值自动发零速（手柄掉线不会失控）；
  4. 有速度与转角限幅，第一次试车可以设得很小。

运行（无需 colcon build，直接跑源码即可）：
    export ROS_DOMAIN_ID=28
    source /opt/ros/humble/setup.bash
    source ~/yahboomcar_ros2_ws/software/library_ws/install/setup.bash
    source ~/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash
    python3 ~/r2_mapping/src/r2_mapping_tools/r2_mapping_tools/ps2_teleop.py

或编译后：
    ros2 run r2_mapping_tools ps2_teleop

常用参数覆盖：
    python3 ps2_teleop.py --ros-args \
      -p button_arm:=11 -p axis_linear:=1 -p axis_steer:=2 \
      -p max_speed:=0.2 -p max_steer:=0.02
"""

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool


class Ps2Teleop(Node):

    def __init__(self):
        super().__init__("ps2_teleop")

        # ---------------- 话题 ----------------
        self.declare_parameter("joy_topic", "/joy")
        self.declare_parameter("cmd_topic", "/cmd_vel")

        # ---------------- 映射（必须按实车核对）----------------
        # 前后：原厂代码用 axes[1]（左摇杆上下）
        self.declare_parameter("axis_linear", 1)
        # 转向：原厂代码用 axes[2]（右摇杆左右）
        self.declare_parameter("axis_steer", 2)
        self.declare_parameter("axis_linear_invert", False)
        self.declare_parameter("axis_steer_invert", False)

        # 解锁键：实测这个手柄能稳定产生 11（蜂鸣器键）。
        # 按一次解锁、再按一次上锁。
        self.declare_parameter("button_arm", 11)
        # 可选：慢速档切换键；-1 表示不用
        self.declare_parameter("button_slow", -1)
        self.declare_parameter("slow_factor", 0.4)

        # ---------------- 安全限制 ----------------
        # 单位 m/s。第一次试车建议 0.15-0.2
        self.declare_parameter("max_speed", 0.25)
        # 转向量纲注意：驱动端 vy = msg.linear.y 直接进 set_car_motion()。
        # 历史记录是 转向角(deg)/1000，即 30° -> 0.03。
        # 若实测发现量纲是别的（例如毫弧度），只改这个值即可。
        self.declare_parameter("max_steer", 0.02)
        # 直行微调，同样是 deg/1000；历史标定值 -2.9° -> -0.0029
        self.declare_parameter("steer_trim", 0.0)
        # 回中"假零值"。
        # 这台车的底盘固件把 linear.y == 0 当成"不更新转向"，
        # 所以松杆时如果真发 0，前轮会停在原地不回正。
        # 用一个小到看不出来的非零值代替 0，强制伺服回到中位。
        # 单位同样是 deg/1000，0.001 = 1°。设 0 可关闭这个行为。
        self.declare_parameter("steer_epsilon", 0.001)
        # 摇杆死区
        self.declare_parameter("deadzone", 0.15)
        # 断流保护：超过这么久没收到 /joy 就发零速并上锁
        self.declare_parameter("watchdog_timeout", 0.5)
        # 控制周期
        self.declare_parameter("publish_rate", 20.0)
        # ---------------- 解锁提示音 ----------------
        # 原厂手柄节点把 buttons[11] 当蜂鸣器开关，按一次就一直响，
        # 而 11 正好是我们的解锁键。用它自己的提示音替代原厂的：
        # 响 N 声然后停（最后一帧显式发 False，保证不会停在"响"的状态）。
        # 注意：前提是原厂节点的 Buzzer 话题已经被 remap 掉，
        #       见 config/yahboomcar_bringup_R2_launch_original.py
        self.declare_parameter("beep_on_arm", True)
        self.declare_parameter("beep_count", 3)
        self.declare_parameter("beep_period_ms", 150.0)

        p = self.get_parameter
        self.joy_topic = p("joy_topic").value
        self.cmd_topic = p("cmd_topic").value
        self.axis_linear = int(p("axis_linear").value)
        self.axis_steer = int(p("axis_steer").value)
        self.linear_invert = bool(p("axis_linear_invert").value)
        self.steer_invert = bool(p("axis_steer_invert").value)
        self.button_arm = int(p("button_arm").value)
        self.button_slow = int(p("button_slow").value)
        self.slow_factor = float(p("slow_factor").value)
        self.max_speed = abs(float(p("max_speed").value))
        self.max_steer = abs(float(p("max_steer").value))
        self.steer_trim = float(p("steer_trim").value)
        self.steer_epsilon = float(p("steer_epsilon").value)
        self.deadzone = abs(float(p("deadzone").value))
        self.watchdog_timeout = float(p("watchdog_timeout").value)
        rate = max(1.0, float(p("publish_rate").value))
        self.beep_on_arm = bool(p("beep_on_arm").value)
        self.beep_count = max(0, int(p("beep_count").value))
        self.beep_period = max(0.05, float(p("beep_period_ms").value) / 1000.0)

        self.armed = False
        self.slow = False
        self.speed_cmd = 0.0
        self.steer_cmd = 0.0
        self.last_joy_time = None
        self._prev_buttons = {}
        self._watchdog_fired = False
        self._beep_left = 0
        self._beep_on = False
        self._beep_next = 0.0

        self.pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self._buzzer_pub = self.create_publisher(Bool, "/Buzzer", 10)
        self.sub = self.create_subscription(Joy, self.joy_topic, self._on_joy, 10)
        self.timer = self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info(
            "ps2_teleop 启动：{} -> {}｜前后=axes[{}] 转向=axes[{}] 解锁=buttons[{}]".format(
                self.joy_topic, self.cmd_topic,
                self.axis_linear, self.axis_steer, self.button_arm,
            )
        )
        self.get_logger().warn(
            "当前为【上锁】状态，不会发任何指令。按按键 {} 解锁。".format(self.button_arm)
        )
        self.get_logger().info(
            "限速 {:.2f} m/s，最大转角 {:.3f}（deg/1000），死区 {:.2f}，"
            "看门狗 {:.1f} s".format(
                self.max_speed, self.max_steer, self.deadzone, self.watchdog_timeout
            )
        )

    # ------------------------------------------------------------------
    def _edge(self, buttons, index):
        """检测按钮 index 的上升沿（0 -> 1）。"""
        if index < 0 or index >= len(buttons):
            return False
        current = int(buttons[index])
        previous = self._prev_buttons.get(index, 0)
        self._prev_buttons[index] = current
        return current == 1 and previous == 0

    @staticmethod
    def _axis(axes, index):
        if index < 0 or index >= len(axes):
            return 0.0
        return float(axes[index])

    def _apply_deadzone(self, value):
        if abs(value) < self.deadzone:
            return 0.0
        # 死区外重新归一化，避免越过死区的瞬间跳变
        sign = 1.0 if value > 0 else -1.0
        return sign * (abs(value) - self.deadzone) / (1.0 - self.deadzone)

    def _on_joy(self, msg):
        self.last_joy_time = time.monotonic()
        if self._watchdog_fired:
            self._watchdog_fired = False
            self.get_logger().info("收到 /joy，看门狗解除。")

        buttons = list(msg.buttons)

        if self._edge(buttons, self.button_arm):
            self.armed = not self.armed
            if self.armed:
                self.get_logger().warn("★ 已解锁，可以遥控（推杆前请确认车已架起或场地清空）")
                self._queue_beep()
            else:
                self.get_logger().warn("☆ 已上锁，停止发送指令")
                self._publish_zero()

        if self._edge(buttons, self.button_slow):
            self.slow = not self.slow
            self.get_logger().info(
                "慢速档：{}（x{:.2f}）".format("开" if self.slow else "关", self.slow_factor)
            )

        speed = self._apply_deadzone(self._axis(msg.axes, self.axis_linear))
        steer = self._apply_deadzone(self._axis(msg.axes, self.axis_steer))
        if self.linear_invert:
            speed = -speed
        if self.steer_invert:
            steer = -steer

        scale = self.slow_factor if self.slow else 1.0
        self.speed_cmd = max(-1.0, min(1.0, speed)) * self.max_speed * scale
        self.steer_cmd = (
            max(-1.0, min(1.0, steer)) * self.max_steer * scale + self.steer_trim
        )
        # 回中补偿：stick 居中时 steer 会被死区打成 0，
        # 直接发 0 固件不认，所以替换成极小非零值。
        if abs(self.steer_cmd) < 1e-12 and abs(self.steer_epsilon) > 0.0:
            self.steer_cmd = self.steer_epsilon

    # ------------------------------------------------------------------
    def _publish_zero(self):
        # 停车时 linear.x 必须为 0；linear.y 用 epsilon 让前轮回正
        # （给 0 固件不认，见 steer_epsilon 参数说明）。
        twist = Twist()
        twist.linear.y = self.steer_epsilon
        self.pub.publish(twist)
        self.speed_cmd = 0.0
        self.steer_cmd = 0.0

    # ------------------------------------------------------------------
    def _queue_beep(self):
        """安排"响 N 声就停"的解锁提示音。"""
        if not self.beep_on_arm or self.beep_count <= 0:
            return
        self._beep_left = self.beep_count
        self._beep_on = False
        self._beep_next = time.monotonic()

    def _service_beep(self):
        """提示音状态机：True -> False 交替，最后一帧一定是 False。"""
        if self._beep_left <= 0:
            return
        now = time.monotonic()
        if now < self._beep_next:
            return
        if not self._beep_on:
            self._buzzer_pub.publish(Bool(data=True))
            self._beep_on = True
        else:
            self._buzzer_pub.publish(Bool(data=False))
            self._beep_on = False
            self._beep_left -= 1
        self._beep_next = now + self.beep_period

    def _tick(self):
        self._service_beep()
        if not self.armed:
            return

        if self.last_joy_time is None:
            self.get_logger().warn("已解锁但从未收到 /joy，保持静止。")
            self._publish_zero()
            return

        idle = time.monotonic() - self.last_joy_time
        if idle > self.watchdog_timeout:
            self._publish_zero()
            self.armed = False
            self._watchdog_fired = True
            self.get_logger().error(
                "看门狗触发：{:.2f} s 没收到 /joy，已发零速并上锁。".format(idle)
            )
            return

        twist = Twist()
        twist.linear.x = self.speed_cmd
        # R2 阿克曼：转向走 linear.y，不是 angular.z
        twist.linear.y = self.steer_cmd
        twist.angular.z = 0.0
        self.pub.publish(twist)


def main(argv=None):
    rclpy.init(args=argv)
    node = Ps2Teleop()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl-C 与 SIGTERM（被 timeout/kill 终止）都走这里，不算错误
        pass
    finally:
        # 退出前尽量补几帧零速。此时上下文可能已经被外部关闭，
        # 发布失败是正常的，不要让它盖住真正的退出原因。
        try:
            if rclpy.ok():
                for _ in range(5):
                    node.pub.publish(Twist())
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

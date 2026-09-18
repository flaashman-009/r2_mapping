#!/usr/bin/env python3
"""直接读取 /dev/input/jsN 的原始事件，绕过 joy_node。

用途：当"推摇杆没反应"时，用来判断问题出在哪一侧。

  - 本脚本能看到摇杆变化 → 手柄和接收器是好的，问题在 ROS 侧
    （joy_node / 话题 / 订阅者）
  - 本脚本也看不到 → 问题在手柄或接收器本身

只依赖标准库，不需要 ROS 环境。可以和多进程同时读（joystick API 支持多读者）。

用法：
    python3 js_raw.py                 # 默认 /dev/input/js0
    python3 js_raw.py /dev/input/js1

输出示例：
    12:03:41  AXIS 1 =  32767 (1.000)
    12:03:42  BTN  11 = 1
"""

import os
import struct
import sys
import time

JS_EVENT_FORMAT = "IhBB"  # time(u32), value(s16), type(u8), number(u8)
JS_EVENT_SIZE = struct.calcsize(JS_EVENT_FORMAT)
JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    path = argv[0] if argv else "/dev/input/js0"

    if not os.path.exists(path):
        print("找不到设备：{}".format(path))
        print("可用设备：")
        for name in sorted(os.listdir("/dev/input")):
            if name.startswith("js"):
                print("    /dev/input/" + name)
        return 1

    print("读取 {}（Ctrl-C 退出）".format(path))
    print("现在推摇杆、按按键，观察下面的输出。")
    print("")

    axes = {}
    buttons = {}
    axis_seen = set()
    button_seen = set()

    try:
        with open(path, "rb") as fh:
            while True:
                data = fh.read(JS_EVENT_SIZE)
                if len(data) < JS_EVENT_SIZE:
                    print("设备关闭或读取出错。")
                    break
                _, value, event_type, number = struct.unpack(JS_EVENT_FORMAT, data)
                is_init = bool(event_type & JS_EVENT_INIT)
                event_type &= ~JS_EVENT_INIT

                if event_type == JS_EVENT_AXIS:
                    axis_seen.add(number)
                    if axes.get(number) != value or is_init:
                        axes[number] = value
                        print("{}  AXIS {} = {:>6}  ({:+.3f}){}".format(
                            time.strftime("%H:%M:%S"), number, value,
                            value / 32767.0, "   [初始]" if is_init else ""))
                elif event_type == JS_EVENT_BUTTON:
                    button_seen.add(number)
                    if buttons.get(number) != value or is_init:
                        buttons[number] = value
                        print("{}  BTN  {} = {}{}".format(
                            time.strftime("%H:%M:%S"), number, value,
                            "   [初始]" if is_init else ""))
    except KeyboardInterrupt:
        print("")
        print("本次看到过的轴：{}".format(sorted(axis_seen) or "无"))
        print("本次看到过的按键：{}".format(sorted(button_seen) or "无"))
    return 0


if __name__ == "__main__":
    sys.exit(main())


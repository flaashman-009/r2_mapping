#!/usr/bin/env bash
# 急停：让底盘真正停下来。
#
# 为什么需要这个脚本（而不是直接在命令行敲 ros2 topic pub）：
#   /cmd_vel 的消息体里有空格，如果从别的终端（尤其是 Windows 的
#   PowerShell 经 ssh）拼一条长命令，引号很容易被吃掉，消息会被拆成
#   多个参数 —— ros2 topic pub 直接报错退出，**指令根本没发出去**。
#   2026-09-16 就是这样：以为发了零速，实际一条都没发，车继续往前跑。
#   所以：凡是带消息体的指令，一律写进脚本文件再执行。
#
# 用法：
#   bash ~/r2_mapping/scripts/estop.sh          # 停
#   bash ~/r2_mapping/scripts/estop.sh --hard   # 停完再把驱动也杀掉

set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
source "${SCRIPT_DIR}/env.sh" >/dev/null 2>&1

HARD=0
[ "${1:-}" = "--hard" ] && HARD=1

echo "=== 急停 ==="

# ---------------------------------------------------------------- 1. 停发布者
echo "[1/4] 停掉所有会发 /cmd_vel 的进程"
for pat in 'controller_[s]erver' 'bt_[n]avigator' 'behavior_[s]erver' \
           'planner_[s]erver' 'smoother_[s]erver' 'ps2_[t]eleop' \
           'nav_[w]atch' 'navigation_r2_[l]aunch'; do
  pkill -f "$pat" 2>/dev/null && echo "    停了：${pat}"
done
sleep 0.5

# ---------------------------------------------------------------- 2. 驱动活着吗
if ! pgrep -f 'Ackman_[d]river' >/dev/null 2>&1; then
  echo "[2/4] 底盘驱动没在跑，先拉起来（否则没东西能转发停止指令）"
  nohup ros2 run yahboomcar_bringup Ackman_driver_R2 \
    >/tmp/estop_drv.log 2>&1 </dev/null &
  sleep 6
else
  echo "[2/4] 底盘驱动在跑"
fi

# ---------------------------------------------------------------- 3. 灌零速
# 注意 linear.y 不能给 0（固件会把 0 当成"不更新转向"），给 0.001 = 1°
echo "[3/4] 持续灌零速 6 秒（50 Hz）"
timeout 7 ros2 topic pub --rate 50 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.001, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
  >/dev/null 2>&1
echo "    已发送"

# ---------------------------------------------------------------- 4. 验证
echo "[4/4] 读底盘反馈确认"
# ros2 topic echo 会把 "---" 分隔符也打出来，所以只取第一个数字
read_fb() {
  timeout 6 ros2 topic echo /vel_raw --once --field "$1" 2>/dev/null \
    | grep -oE '^-?[0-9]+(\.[0-9]+)?' | head -1
}
VX="$(read_fb linear.x)"
STEER="$(read_fb linear.y)"
echo "    速度 feedback vx = ${VX:-取不到}   （0.0 = 已停）"
echo "    转角 feedback   = ${STEER:-取不到} 度"
if [ "${VX}" = "0" ] || [ "${VX}" = "0.0" ]; then
  echo "    ✓ 底盘已停"
else
  echo "    ✗ 反馈不是 0！**立刻拍电源开关**"
fi

if [ "${HARD}" = "1" ]; then
  echo "[hard] 杀掉驱动，释放串口"
  bash "${SCRIPT_DIR}/restart_hardware.sh" --no-start
fi

echo "=== 结束 ==="
echo "提醒：软件急停只能保证'不再发新指令'，不能切断电机供电。"
echo "      真出事请直接拍电源开关。"

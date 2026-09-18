#!/usr/bin/env bash
# 快速验证 cmd_vel_ackermann：一边持续发指令，一边看它算出什么、输出什么。
# 用法： bash ack_quick.sh <v> <w>

set -o pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/env.sh" >/dev/null 2>&1

V="${1:-0.23}"
W="${2:-0.5}"

pkill -f "test_[a]ckermann" 2>/dev/null
pkill -f "cmd_vel_[a]ckermann" 2>/dev/null
sleep 1

nohup python3 -u "${SCRIPT_DIR}/cmd_vel_ackermann.py" \
  --ros-args -p log_every:=10 >/tmp/ack_q.log 2>&1 </dev/null &
sleep 4

MSG="$(printf '{linear: {x: %s, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: %s}}' "$V" "$W")"
echo "输入: v=$V  w=$W"
echo "发出: $MSG"

timeout 7 ros2 topic pub --rate 20 /cmd_vel_nav geometry_msgs/msg/Twist \
  "$MSG" >/dev/null 2>&1 &
PUB=$!

sleep 4
echo
echo "--- 稳态时 /cmd_vel 的内容 ---"
timeout 3 ros2 topic echo /cmd_vel --once 2>&1 | head -10

echo
echo "--- 稳态时底盘反馈 /vel_raw.linear.y（实际转角，度）---"
FB="$(timeout 3 ros2 topic echo /vel_raw --once --field linear.y 2>/dev/null \
      | grep -oE '^-?[0-9]+(\.[0-9]+)?' | head -1)"
echo "    实际转角 = ${FB:-取不到} 度"

kill "$PUB" 2>/dev/null
sleep 0.5

echo
echo "--- 适配节点日志（每 10 条打一行）---"
tail -6 /tmp/ack_q.log

pkill -f "cmd_vel_[a]ckermann" 2>/dev/null
echo "（适配节点已停）"

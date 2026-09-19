#!/usr/bin/env bash
# 用适配节点自己的日志验证"低速限幅"。
# 让它每收到一条指令打一行（log_every=1），我们直接看它算出的 delta。

set -o pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/env.sh" >/dev/null 2>&1

IN=/cmd_vel_tin
OUT=/cmd_vel_tout
LOG=/tmp/ack_lowspeed.log

pkill -f "input_topic:=${IN}" 2>/dev/null
sleep 1
rm -f "$LOG"
nohup python3 -u "${SCRIPT_DIR}/cmd_vel_ackermann.py" --ros-args \
  -p "input_topic:=${IN}" -p "output_topic:=${OUT}" \
  -p wheelbase:=0.2681 -p max_steer_deg:=40.0 \
  -p low_speed:=0.12 -p low_speed_steer_deg:=20.0 \
  -p reverse_steer_deg:=22.0 \
  -p log_every:=20 \
  >"$LOG" 2>&1 </dev/null &
sleep 4

send() {   # send <v> <w> <秒数>
  local m
  m="$(printf '{linear: {x: %s, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: %s}}' "$1" "$2")"
  timeout "$3" ros2 topic pub --rate 20 "$IN" geometry_msgs/msg/Twist "$m" >/dev/null 2>&1
}

echo "=== 依次发送，看适配节点自己算出的 delta ==="
for spec in "0.05 0.45" "0.08 0.45" "0.12 0.45" "0.23 0.45" "0.23 0.90"; do
  set -- $spec
  echo ""
  echo "--- 发送 v=$1  ω=$2 ---"
  send "$1" "$2" 4
  sleep 0.5
done

echo ""
echo "=== 适配节点日志（每 20 条打一行）==="
grep -o 'v=[+-][0-9.]* w=[+-][0-9.]* -> delta=[+-][0-9.]* deg' "$LOG" | sort -u
echo ""
echo "（理论值：v=0.05→67°、v=0.08→56°、v=0.12→45°、v=0.23→28°、ω=0.9→46°；"
echo "  低速限幅后应分别被压到 ≈28°、≈29°、40°、28°、40°）"

pkill -f "input_topic:=${IN}" 2>/dev/null
echo ""
echo "（测试节点已停）"

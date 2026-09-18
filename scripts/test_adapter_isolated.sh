#!/usr/bin/env bash
# 完全隔离地测试适配节点：输入输出都走测试话题，真车一点不动。
#
# 验证四件事：
#   1. ω -> δ 换算对不对
#   2. 限幅（±40°）生效
#   3. 死区（变化 <1.5° 不动）
#   4. 看门狗叫停后是否回正、解除后是否恢复

set -o pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/env.sh" >/dev/null 2>&1

IN=/cmd_vel_testin
OUT=/cmd_vel_testout

pkill -f "cmd_vel_ackermann.py --ros-args -p input_topic:=${IN}" 2>/dev/null
sleep 1
nohup python3 -u "${SCRIPT_DIR}/cmd_vel_ackermann.py" --ros-args \
  -p "input_topic:=${IN}" -p "output_topic:=${OUT}" \
  -p wheelbase:=0.2681 -p max_steer_deg:=40.0 -p max_steer_rate_dps:=45.0 \
  -p steer_deadband_deg:=1.5 -p log_every:=0 \
  >/tmp/adapter_iso.log 2>&1 </dev/null &
sleep 4

pub() {   # pub <v> <w>  —— 持续发 5 秒
  local m
  m="$(printf '{linear: {x: %s, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: %s}}' "$1" "$2")"
  timeout 5 ros2 topic pub --rate 20 "$IN" geometry_msgs/msg/Twist "$m" >/dev/null 2>&1
}
halt() {
  timeout 3 ros2 topic pub --once /cmd_vel_halt std_msgs/msg/Bool "{data: $1}" \
    >/dev/null 2>&1
}
read_out() {
  timeout 3 ros2 topic echo "$OUT" --once --field linear.y 2>/dev/null \
    | grep -oE '^-?[0-9]+(\.[0-9]+)?' | head -1
}

echo "=============================================================="
echo " 适配节点隔离测试（输出到 ${OUT}，没有订阅者，车不会动）"
echo "=============================================================="
printf '%-26s %-14s %-14s\n' "输入" "期望" "实际输出(度)"

check() {
  pub "$1" "$2" &
  sleep 3.2
  d="$(read_out)"
  deg="$(echo "${d:-0}" | awk '{printf "%.1f", $1*1000}')"
  printf '%-26s %-14s %-14s\n' "v=$1 ω=$2" "$3" "${deg}"
  wait
}

check 0.23 0.5    "atan(0.5L/v)=30"
check 0.23 -0.5   "-30"
check 0.23 2.0    "限幅 40"
check 0.23 -2.0   "限幅 -40"
check 0.23 0.0    "0 -> 强制1"

echo
echo "--- 看门狗叫停 ---"
pub 0.23 0.5 &
P=$!
sleep 3.5
echo "  叫停前输出: $(read_out)"
halt true
sleep 3
echo "  叫停后输出: $(read_out)   <- 应该回到约 0.001（1度）"
halt false
sleep 3.5
echo "  解除后输出: $(read_out)   <- 应该回到约 0.030（30度）"
kill $P 2>/dev/null

echo
echo "--- 适配节点统计 ---"
tail -3 /tmp/adapter_iso.log
pkill -f "cmd_vel_ackermann.py --ros-args -p input_topic:=${IN}" 2>/dev/null
echo "（隔离测试节点已停，真车没有收到任何指令）"

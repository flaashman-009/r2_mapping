#!/usr/bin/env bash
# 实测底盘对 angular.z 的解释，为转向适配节点定标
#
# 要回答两个问题：
#   1. angular.z -> 实际转角，系数是多少？（度 / 单位）
#   2. angular.z = 0 会不会让前轮回正？（还是"保持上一次"）
#
# 安全性：全程 linear.x = 0，车不会走，只会动前轮。请让开前轮。

set -o pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/env.sh" >/dev/null 2>&1

send() {
  local w="$1"
  # 用脚本文件，避免引号被外层 shell 吃掉
  timeout 4 ros2 topic pub --rate 20 -t 30 /cmd_vel \
    geometry_msgs/msg/Twist \
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: ${w}}" \
    >/dev/null 2>&1
}

read_steer() {
  timeout 5 ros2 topic echo /vel_raw --once --field linear.y 2>/dev/null \
    | grep -oE '^-?[0-9]+(\.[0-9]+)?' | head -1
}

echo "=============================================="
echo " 转向通道定标（车不会动，只动前轮）"
echo "=============================================="

if ! pgrep -f 'Ackman_[d]river' >/dev/null 2>&1; then
  echo "底盘驱动没在跑，先拉起来 ..."
  nohup ros2 run yahboomcar_bringup Ackman_driver_R2 \
    >/tmp/steer_probe_drv.log 2>&1 </dev/null &
  sleep 6
fi

printf '%-14s %-16s %-18s\n' "指令 angular.z" "实际转角(度)" "说明"
for w in 0.2 0.4 0.6 0.8 1.0 -0.4 -0.8; do
  send "$w"
  sleep 0.6
  s="$(read_steer)"
  printf '%-14s %-16s %-18s\n' "$w" "${s:-取不到}" "比值 $(python3 -c "
try:
    print('%.1f' % (float('${s}')/float('${w}')))
except Exception:
    print('-')")"
done

echo
echo "--- 关键：发 0 会不会回正 ---"
send 0.6
sleep 0.6
echo "  先给 0.6，实际转角 = $(read_steer) 度"
send 0.0
sleep 0.6
echo "  再给 0.0，实际转角 = $(read_steer) 度   <- 如果还是 -20 多度，说明 0=保持不更新"
send -0.02
sleep 0.6
echo "  再给 -0.02，实际转角 = $(read_steer) 度"
send 0.02
sleep 0.6
echo "  最后给 +0.02 回正，实际转角 = $(read_steer) 度"
echo "=============================================="

#!/usr/bin/env bash
# 干净重启底盘硬件（底盘 + 雷达 + TF + IMU + EKF）。
#
# 固化了两类踩过的坑：
#   1. 强杀 ROS 进程后 /dev/shm 里会残留 fastrtps 共享内存段，
#      导致新进程报 "Failed init_port ... open_and_lock_file failed"
#   2. 网络切换（换 WiFi / IP 变了）后，已经在跑的节点仍绑在旧接口上，
#      新进程加入不了 ROS 图（ros2 node list 一片空白）
#
# 用法：
#   ./restart_hardware.sh            # 重启
#   ./restart_hardware.sh --no-start # 只停不启

set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
source "${SCRIPT_DIR}/env.sh" >/dev/null 2>&1

PATTERNS=(
  Ackman_driver_R2
  base_node_R2
  imu_filter_madgwick
  ekf_filter_node
  yahboom_joy_R2
  ydlidar_ros2_driver_node
  # joy_node 必须一起清：硬件 bringup 每次都起一个，
  # 漏掉的话会累积多个，导致 /joy 有多份重复消息
  joy_node
  slam_gmapping
  scan_filter_node
  # 实验版里程计（替代 base_node_R2 时用）。一起清掉，
  # 保证重启后回到"原厂 base_node_R2 单独发 /odom_raw"的干净状态。
  r2_odom
  laser_bringup_launch
  # 这两个 launch 内部都包含硬件 bringup，停硬件时必须一起停，
  # 否则会出现两个驱动抢 /dev/myserial
  mapping_gmapping.launch
  map_gmapping_4ros_launch
  mapping_slam_toolbox
  # Nav2 也一起清：它虽然不占串口，但留着会一直往 /cmd_vel 发指令
  navigation_teb_launch
  navigation_dwa_launch
  nav2_container
)

stop_all() {
  local sig="$1"
  local pat pids
  for pat in "${PATTERNS[@]}"; do
    pids="$(pgrep -f "$pat" 2>/dev/null || true)"
    if [[ -n "$pids" ]]; then
      # shellcheck disable=SC2086
      kill "-${sig}" ${pids} 2>/dev/null || true
    fi
  done
}

echo "[1/4] 停止现有硬件 ..."
stop_all TERM
sleep 4
stop_all KILL
sleep 2

left="$(pgrep -f ydlidar_ros2_driver_node 2>/dev/null || true)"
if [[ -n "$left" ]]; then
  echo "  警告：还有进程没停掉：${left}"
else
  echo "  已全部停止"
fi

echo "[2/4] 清理 fastrtps 共享内存 ..."
before="$(ls /dev/shm/ 2>/dev/null | grep -c fastrtps || true)"
rm -f /dev/shm/fastrtps_* 2>/dev/null || true
after="$(ls /dev/shm/ 2>/dev/null | grep -c fastrtps || true)"
echo "  共享内存段：${before} -> ${after}"

echo "[3/4] 停掉 ros2 daemon ..."
timeout 10 ros2 daemon stop >/dev/null 2>&1 || true
echo "  已停"

if [[ "${1:-}" == "--no-start" ]]; then
  echo "按 --no-start 要求，不重启。"
  exit 0
fi

echo "[4/4] 启动硬件（约 22 秒）..."
LOG=/tmp/r2_hw.log
rm -f "${LOG}"
nohup setsid ros2 launch yahboomcar_nav laser_bringup_launch.py \
  > "${LOG}" 2>&1 < /dev/null &
sleep 22

echo "--- 节点 ---"
timeout 15 ros2 node list 2>&1 || true
echo "--- /odom ---"
timeout 8 ros2 topic hz /odom 2>&1 | head -2 || true
echo "--- /scan ---"
timeout 8 ros2 topic hz /scan 2>&1 | head -2 || true
echo "--- 日志尾 ---"
tail -3 "${LOG}" 2>/dev/null || true
echo ""
echo "完整日志：${LOG}"

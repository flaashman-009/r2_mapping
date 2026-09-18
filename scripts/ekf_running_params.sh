#!/usr/bin/env bash
# 核对**运行中**的 EKF 参数（不是文件里的，是节点实际加载的）
set -o pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/env.sh" >/dev/null 2>&1

N=/ekf_filter_node
echo "===== 运行中的 EKF 参数 ====="
for p in frequency sensor_timeout two_d_mode publish_tf \
         odom_frame base_link_frame world_frame \
         odom0 imu0 imu0_relative odom0_differential \
         imu0_remove_gravitational_acceleration print_diagnostics; do
  printf '%-42s ' "${p}"
  ros2 param get "${N}" "${p}" 2>&1 | tail -1
done

echo
echo "----- 15 位 config 数组（顺序: x y z roll pitch yaw vx vy vz vroll vpitch vyaw ax ay az）-----"
for p in odom0_config imu0_config; do
  echo "${p}:"
  ros2 param get "${N}" "${p}" 2>&1 | tail -1
done

echo
echo "===== 实际话题频率 ====="
for t in /odom /odom_raw /imu/data /imu/data_raw /scan; do
  printf '%-16s ' "${t}"
  timeout 8 ros2 topic hz "${t}" 2>/dev/null | grep -m1 average || echo "(无数据)"
done

echo
echo "===== 文件里写的 frequency ====="
grep -n -m2 -E '^\s*frequency:' \
  ~/yahboomcar_ros2_ws/software/library_ws/install/robot_localization/share/robot_localization/params/ekf_x1_x3.yaml 2>/dev/null \
  || echo "  (找不到 install 里的 ekf_x1_x3.yaml)"

#!/usr/bin/env bash
# 确认本次 AMCL 调参在运行中的节点上真的生效了

set -o pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/env.sh" >/dev/null 2>&1

echo "=============================================="
echo " AMCL 调参生效确认"
echo "=============================================="

for p in max_beams update_min_d update_min_a \
         recovery_alpha_fast recovery_alpha_slow \
         scan_topic laser_max_range; do
  printf '  %-22s ' "$p"
  ros2 param get /amcl "$p" 2>/dev/null | tail -1
done

echo
echo "  转向适配节点："
for p in max_steer_deg reverse_steer_deg max_steer_rate_dps steer_deadband_deg; do
  printf '    %-22s ' "$p"
  ros2 param get /cmd_vel_ackermann "$p" 2>/dev/null | tail -1
done

echo
echo "  定位护卫："
for p in cov_stop t_stop cov_ok; do
  printf '    %-22s ' "$p"
  ros2 param get /localization_guard "$p" 2>/dev/null | tail -1
done

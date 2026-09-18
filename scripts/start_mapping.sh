#!/usr/bin/env bash
# 一键建图：硬件 bringup + slam_toolbox online_async（+ RViz）。
#
# 用法：
#   ./scripts/start_mapping.sh
#   ./scripts/start_mapping.sh rviz:=false
#   ./scripts/start_mapping.sh start_vendor:=false        # 硬件已在跑
#   ./scripts/start_mapping.sh use_scan_filter:=true
#
# 注意：建图与定位不能同时运行（map->odom 只能有一个发布者）。

set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
source "${SCRIPT_DIR}/env.sh"

if ! ros2 pkg prefix r2_mapping_bringup >/dev/null 2>&1; then
  echo "[start_mapping] 找不到 r2_mapping_bringup，先编译：" >&2
  echo "    cd \"${R2_MAPPING_ROOT}\" && colcon build --symlink-install" >&2
  exit 2
fi

echo "[start_mapping] ROS_DOMAIN_ID=${ROS_DOMAIN_ID}  R2_MAPPING_ROOT=${R2_MAPPING_ROOT}"
exec ros2 launch r2_mapping_bringup mapping.launch.py "$@"


#!/usr/bin/env bash
# 一键定位复测：硬件 bringup + map_server + AMCL（+ RViz）。
#
# 用法：
#   ./scripts/start_localization.sh                       # 默认 maps/room_01.yaml
#   ./scripts/start_localization.sh maps/room_02.yaml
#   ./scripts/start_localization.sh maps/room_02.yaml start_vendor:=false
#
# ⚠️ 启动前请先停掉 slam_toolbox，否则 map->odom 会有两个发布者。

set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
source "${SCRIPT_DIR}/env.sh"

MAP_ARG="${1:-${R2_MAPPING_ROOT}/maps/room_01.yaml}"
shift || true

# 允许传相对 maps/ 的路径
if [[ ! -f "${MAP_ARG}" && -f "${R2_MAPPING_ROOT}/${MAP_ARG}" ]]; then
  MAP_ARG="${R2_MAPPING_ROOT}/${MAP_ARG}"
fi

if ! ros2 pkg prefix r2_mapping_bringup >/dev/null 2>&1; then
  echo "[start_localization] 找不到 r2_mapping_bringup，先编译：" >&2
  echo "    cd \"${R2_MAPPING_ROOT}\" && colcon build --symlink-install" >&2
  exit 2
fi

echo "[start_localization] 地图：${MAP_ARG}"
exec ros2 launch r2_mapping_bringup localization.launch.py map:="${MAP_ARG}" "$@"


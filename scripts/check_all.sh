#!/usr/bin/env bash
# 建图前置条件自检（Gate A）。所有参数原样透传给 health_check。
#
# 用法：
#   ./scripts/check_all.sh
#   ./scripts/check_all.sh --duration 15
#   ./scripts/check_all.sh --expect-map          # 定位复测模式
#   ./scripts/check_all.sh --json

set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
source "${SCRIPT_DIR}/env.sh"

if ! ros2 pkg prefix r2_mapping_tools >/dev/null 2>&1; then
  echo "[check_all] 找不到 r2_mapping_tools，先编译：" >&2
  echo "    cd \"${R2_MAPPING_ROOT}\" && source /opt/ros/humble/setup.bash && colcon build --symlink-install" >&2
  exit 2
fi

exec ros2 run r2_mapping_tools health_check "$@"


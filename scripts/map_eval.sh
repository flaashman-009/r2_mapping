#!/usr/bin/env bash
# 地图评估包装：优先用已安装的 r2_mapping_tools，未编译时直接跑源码。
#
# 用法：
#   ./scripts/map_eval.sh maps/room_01.yaml
#   ./scripts/map_eval.sh maps/room_01.yaml --compare maps/room_02.yaml

set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SOURCE_PY="${SCRIPT_DIR}/../src/r2_mapping_tools/r2_mapping_tools/map_eval.py"

if command -v ros2 >/dev/null 2>&1 && ros2 pkg prefix r2_mapping_tools >/dev/null 2>&1; then
  # shellcheck source=/dev/null
  source "${SCRIPT_DIR}/env.sh"
  exec ros2 run r2_mapping_tools map_eval "$@"
fi

# 没有 ROS 环境也能用（只依赖 numpy + PyYAML）
exec python3 "${SOURCE_PY}" "$@"


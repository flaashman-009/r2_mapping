#!/usr/bin/env bash
# 录制建图/定位过程的 rosbag。
#
# 用法：
#   ./scripts/record_bag.sh                       # 自动命名，录 120 s
#   ./scripts/record_bag.sh room_01_run1          # 指定名字
#   ./scripts/record_bag.sh room_01_run1 300      # 指定时长（秒）
#
# Ctrl-C 可以提前结束（会正常收尾，不会损坏 bag）。

set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
source "${SCRIPT_DIR}/env.sh"

NAME="${1:-mapping_$(date +%Y%m%d_%H%M%S)}"
DURATION="${2:-120}"
OUT_DIR="${R2_MAPPING_ROOT}/bags"
OUT="${OUT_DIR}/${NAME}"

mkdir -p "${OUT_DIR}"

available_kb="$(df -Pk "${OUT_DIR}" | awk 'NR==2 {print $4}')"
echo "[record_bag] 目标目录剩余 $((available_kb / 1024)) MB"
if [[ "${available_kb}" -lt 512000 ]]; then
  echo "[record_bag] ⚠️ 剩余空间不足 500 MB，先清理 ~/.ros/log 再录。" >&2
fi

if [[ -e "${OUT}" ]]; then
  echo "[record_bag] 目录已存在：${OUT}（换一个名字，或先删掉它）" >&2
  exit 1
fi

# 用正则一次覆盖所有关心的话题；不存在的话题不会导致录制失败。
TOPIC_REGEX='/scan.*|/odom.*|/imu.*|/tf|/tf_static|/cmd_vel|/vel_raw|/joint_states|/map.*|/amcl_pose|/initialpose'

echo "[record_bag] 录制 ${DURATION}s -> ${OUT}"
echo "[record_bag] 话题正则：${TOPIC_REGEX}"
echo "[record_bag] Ctrl-C 提前结束"

timeout --signal=INT "${DURATION}" \
  ros2 bag record --regex -o "${OUT}" "${TOPIC_REGEX}"
rc=$?

if [[ "${rc}" -ne 0 && "${rc}" -ne 124 && "${rc}" -ne 130 ]]; then
  echo "[record_bag] 录制异常退出（rc=${rc}）" >&2
  exit "${rc}"
fi

echo ""
echo "[record_bag] 完成。查看内容："
echo "    ros2 bag info ${OUT}"


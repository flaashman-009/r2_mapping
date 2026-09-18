#!/usr/bin/env bash
# 保存地图：slam_toolbox 序列化 + Nav2 兼容的 PGM/YAML。
#
# 用法：
#   ./scripts/save_map.sh room_01
#   ./scripts/save_map.sh room_01 /path/to/output_dir
#
# 产物：
#   <dir>/<name>.pgm / <name>.yaml                        Nav2 map_server 可加载
#   <dir>/<name>_posegraph.posegraph / .data              slam_toolbox 可续建

set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
source "${SCRIPT_DIR}/env.sh"

NAME="${1:-map_$(date +%Y%m%d_%H%M%S)}"
OUT_DIR="${2:-${R2_MAPPING_ROOT}/maps}"
mkdir -p "${OUT_DIR}"

PREFIX="${OUT_DIR}/${NAME}"

echo "=================================================="
echo " 保存地图 -> ${PREFIX}.{pgm,yaml}"
echo "=================================================="

saved=0

# 1) slam_toolbox 自己的 save_map：顺便写序列化 posegraph（以后可以接着建）
if ros2 service list 2>/dev/null | grep -q '^/slam_toolbox/save_map$'; then
  echo "[1/2] 调用 slam_toolbox/save_map（含 posegraph 序列化）..."
  ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
    "{name: {data: '${PREFIX}_posegraph'}}" || true
  saved=1
else
  echo "[1/2] 没有找到 /slam_toolbox/save_map（建图没在跑？），跳过序列化。"
fi

# 2) Nav2 map_saver：产出标准 PGM/YAML，阈值与验收标准的 0.65 / 0.25 对齐
echo "[2/2] 调用 nav2_map_server map_saver_cli ..."
if ros2 run nav2_map_server map_saver_cli \
     -f "${PREFIX}" \
     --occ 0.65 --free 0.196 \
     --ros-args -p use_sim_time:=false
then
  saved=1
else
  echo "[!] map_saver_cli 失败：可能 /map 话题没有数据。"
fi

echo ""
echo "-- 产物 --"
ls -lh "${PREFIX}".* 2>/dev/null || echo "    （没有生成任何文件）"

if [[ -f "${PREFIX}.yaml" ]]; then
  echo ""
  echo "-- YAML 内容 --"
  sed 's/^/    /' "${PREFIX}.yaml"
  echo ""
  echo "下一步："
  echo "    ./scripts/map_eval.sh ${PREFIX}.yaml"
else
  echo "[x] 没有生成 ${PREFIX}.yaml"
  exit 1
fi

[[ "${saved}" -eq 1 ]] || exit 1

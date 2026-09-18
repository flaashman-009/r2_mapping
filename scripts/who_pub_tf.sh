#!/usr/bin/env bash
# 谁在发 odom->base_footprint？
# 把每个节点的发布/订阅列出来，重点看 /tf 和 /odom。
set -o pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/env.sh" >/dev/null 2>&1

for n in "$@"; do
  echo "########## ${n}"
  ros2 node info "${n}" 2>/dev/null
  echo
done

echo "########## /odom 的发布者"
ros2 topic info /odom
echo
echo "########## /odom_raw 的发布者"
ros2 topic info /odom_raw
echo
echo "########## /tf 的发布者（逐个节点确认）"
for n in $(ros2 node list 2>/dev/null); do
  info="$(ros2 node info "${n}" 2>/dev/null)"
  if printf '%s\n' "${info}" | grep -qE '^\s+/tf: '; then
    echo "  [发 /tf]     ${n}"
  fi
  if printf '%s\n' "${info}" | grep -qE '^\s+/tf_static: '; then
    echo "  [发 /tf_static] ${n}"
  fi
done

#!/usr/bin/env bash
# 导航起来之后，验证话题链路是不是按设计接好了

set -o pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/env.sh" >/dev/null 2>&1

echo "=============================================="
echo " 导航链路实时校验"
echo "=============================================="

echo
echo "[1] 关键节点"
for n in amcl controller_server behavior_server map_server; do
  if timeout 6 ros2 node list 2>/dev/null | grep -qx "/$n"; then
    echo "    OK   /$n"
  else
    echo "    !!   缺 /$n"
  fi
done
if pgrep -f "cmd_vel_[a]ckermann" >/dev/null; then
  echo "    OK   cmd_vel_ackermann（进程）"
else
  echo "    !!   缺 cmd_vel_ackermann"
fi
if pgrep -f "scan_[f]ilter_node" >/dev/null; then
  echo "    OK   scan_filter_node（进程）"
else
  echo "    !!   缺 scan_filter_node"
fi

echo
echo "[2] /scan_filtered（导航专用扫描）"
if timeout 6 ros2 topic list 2>/dev/null | grep -qx "/scan_filtered"; then
  echo "    存在，频率："
  timeout 8 ros2 topic hz /scan_filtered 2>/dev/null | grep -m1 average \
    | sed 's/^/      /'
else
  echo "    !! 不存在 —— 扫描滤波没起来，AMCL 会读不到数据"
fi

echo
echo "[3] /cmd_vel_nav（Nav2 输出）"
ros2 topic info /cmd_vel_nav 2>/dev/null | sed 's/^/    /'

echo
echo "[4] /cmd_vel（驱动输入，应只有适配节点在发）"
ros2 topic info /cmd_vel 2>/dev/null | sed 's/^/    /'

echo
echo "[5] AMCL 实际用的扫描话题"
A="$(ros2 param get /amcl scan_topic 2>/dev/null | awk '{print $NF}')"
echo "    scan_topic = ${A}"
if [ "${A}" = "scan_filtered" ]; then
  echo "    OK 与建图一致"
else
  echo "    !! 不是 scan_filtered，又和建图不一致了"
fi

echo
echo "[6] TF 链"
for pair in "map base_footprint" "odom base_footprint"; do
  set -- $pair
  if timeout 8 ros2 run tf2_ros tf2_echo "$1" "$2" 2>/dev/null \
       | grep -q "Translation"; then
    echo "    OK   $1 -> $2"
  else
    echo "    !!   $1 -> $2 查不到（导航前要先设初始位姿）"
  fi
done

echo
echo "[7] 转向适配节点参数"
ros2 param get /cmd_vel_ackermann wheelbase 2>/dev/null | sed 's/^/    /'
ros2 param get /cmd_vel_ackermann max_steer_deg 2>/dev/null | sed 's/^/    /'

echo
echo "=============================================="

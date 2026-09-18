#!/usr/bin/env bash
# 检查有没有重复启动的 Nav2 节点（重复会让指令互相打架）

echo "=============================================="
echo " 重复节点检查"
echo "=============================================="

declare -A WANT=(
  ["amcl"]=1
  ["controller_server"]=1
  ["planner_server"]=1
  ["behavior_server"]=1
  ["bt_navigator"]=1
  ["map_server"]=1
  ["lifecycle_manager"]=2
  ["cmd_vel_ackermann"]=1
  ["scan_filter_node"]=1
)

bad=0
for name in amcl controller_server planner_server behavior_server \
            bt_navigator map_server lifecycle_manager \
            cmd_vel_ackermann scan_filter_node; do
  # 用 pgrep 按命令行计数；pgrep 不会匹配自己（用 grep -c 会把自己算进去）
  n="$(pgrep -fc "/${name}" 2>/dev/null || echo 0)"
  want="${WANT[$name]}"
  if [ "$n" -eq "$want" ]; then
    printf '  OK   %-22s %s 个\n' "$name" "$n"
  else
    printf '  !!   %-22s %s 个（应为 %s）\n' "$name" "$n" "$want"
    bad=1
  fi
done

echo
if [ "$bad" -eq 0 ]; then
  echo "  没有重复，干净。"
else
  echo "  有重复节点！跑 bash nav.sh stop 再重新启动。"
fi

echo
echo "--- Nav2 节点进程总数 ---"
pgrep -cf "/opt/ros/humble/lib/nav2_" 2>/dev/null

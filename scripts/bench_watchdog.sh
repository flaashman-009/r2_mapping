#!/usr/bin/env bash
# 测量迷失看门狗的实际资源占用（新版 vs 旧版思路）
#
# 用法：
#   bash bench_watchdog.sh           # 测新版（只用协方差+幻影运动）
#   bash bench_watchdog.sh residual  # 测开了点云残差的版本

set -o pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/env.sh" >/dev/null 2>&1

EXTRA=""
if [ "${1:-}" = "residual" ]; then
  EXTRA="-p use_residual:=true -p scan_topic:=/scan_filtered"
  echo "=== 模式：开启点云残差（旧思路）==="
else
  echo "=== 模式：低成本版（默认）==="
fi

nohup python3 -u "${SCRIPT_DIR}/lost_watchdog.py" --ros-args \
  -p halt_topic:=/cmd_vel_halt $EXTRA \
  >/tmp/wd_bench.log 2>&1 </dev/null &
sleep 20                      # 跳过启动期

PID="$(pgrep -f 'lost_[w]atchdog' | head -1)"
if [ -z "$PID" ]; then
  echo "看门狗没起来，看日志："; cat /tmp/wd_bench.log; exit 1
fi

# 采样 15 秒算平均 CPU
read -r u1 s1 < <(awk '{print $14, $15}' /proc/$PID/stat)
read -r _ < /dev/null
HZ="$(getconf CLK_TCK)"
T0="$(date +%s%N)"
sleep 15
read -r u2 s2 < <(awk '{print $14, $15}' /proc/$PID/stat)
T1="$(date +%s%N)"

CPU="$(awk -v u1="$u1" -v s1="$s1" -v u2="$u2" -v s2="$s2" \
  -v hz="$HZ" -v t0="$T0" -v t1="$T1" \
  'BEGIN{printf "%.2f", ((u2+s2)-(u1+s1))/hz/((t1-t0)/1e9)*100}')"
RSS="$(awk '/VmRSS/{print $2" "$3}' /proc/$PID/status)"
THREADS="$(awk '/Threads/{print $2}' /proc/$PID/status)"

echo
echo "  进程 PID        : $PID"
echo "  平均 CPU        : ${CPU} %"
echo "  常驻内存 RSS    : ${RSS:-?}"
echo "  线程数          : ${THREADS:-?}"
echo
echo "--- 启动日志 ---"
head -5 /tmp/wd_bench.log
pkill -f 'lost_[w]atch'
echo "(已停)"

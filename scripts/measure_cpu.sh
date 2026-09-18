#!/usr/bin/env bash
# 测量"已经在跑"的某个进程的稳态 CPU（跳过启动期）
# 用法： bash measure_cpu.sh <进程名关键字> [秒数]

pat="${1:?用法: measure_cpu.sh <关键字> [秒数]}"
secs="${2:-15}"

mapfile -t PIDS < <(pgrep -f "$pat")
if [ "${#PIDS[@]}" -eq 0 ]; then
  echo "没找到进程：$pat"
  exit 1
fi

HZ="$(getconf CLK_TCK)"
T0="$(date +%s%N)"
declare -A U1 S1
for p in "${PIDS[@]}"; do
  read -r u s < <(awk '{print $14, $15}' "/proc/$p/stat")
  U1[$p]="$u"; S1[$p]="$s"
done

echo "采样 ${secs} 秒 ..."
sleep "$secs"

T1="$(date +%s%N)"
DT="$(awk -v a="$T0" -v b="$T1" 'BEGIN{print (b-a)/1e9}')"

printf '  %-8s %-10s %-12s %s\n' "PID" "CPU%" "RSS" "命令"
for p in "${PIDS[@]}"; do
  [ -r "/proc/$p/stat" ] || continue
  read -r u2 s2 < <(awk '{print $14, $15}' "/proc/$p/stat")
  cpu="$(awk -v u1="${U1[$p]}" -v s1="${S1[$p]}" -v u2="$u2" -v s2="$s2" \
    -v hz="$HZ" -v dt="$DT" 'BEGIN{printf "%.2f", ((u2+s2)-(u1+s1))/hz/dt*100}')"
  rss="$(awk '/VmRSS/{printf "%.1f MB", $2/1024}' "/proc/$p/status")"
  cmd="$(tr '\0' ' ' < "/proc/$p/cmdline" | cut -c1-60)"
  printf '  %-8s %-10s %-12s %s\n' "$p" "$cpu" "$rss" "$cmd"
done

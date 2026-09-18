#!/usr/bin/env bash
# 监听 USB 事件，用来判断"拔插雷达时系统到底有没有反应"
# 用法： bash usb_watch.sh [秒数]

SECS="${1:-300}"
LOG=/tmp/usb_watch.log

echo "开始监听 USB 事件 ${SECS} 秒，日志写到 ${LOG}"
echo "现在去拔插雷达。"

: >"$LOG"
{
  echo "=== 起始快照 $(date +%T) ==="
  lsusb
  echo
  echo "=== 开始监听 ==="
} >>"$LOG" 2>&1

# 内核事件 + udev 事件双保险
timeout "$SECS" udevadm monitor --kernel --udev --subsystem-match=usb \
  >>"$LOG" 2>&1 &
MON=$!

END=$((SECONDS + SECS))
while [ "$SECONDS" -lt "$END" ]; do
  # 每 5 秒记一次串口设备快照，看有没有新的 ttyUSB 冒出来
  printf '%s  ttyUSB: %s\n' "$(date +%T)" \
    "$(ls /dev/ttyUSB* 2>/dev/null | tr '\n' ' ')" >>"$LOG"
  sleep 5
done

kill "$MON" 2>/dev/null
{
  echo
  echo "=== 结束快照 $(date +%T) ==="
  lsusb
  ls -l /dev/ttyUSB* /dev/ydlidar 2>&1
} >>"$LOG" 2>&1

echo "监听结束，日志：${LOG}"

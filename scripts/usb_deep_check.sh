#!/usr/bin/env bash
# 深度排查：外设（雷达/摄像头/语音）为什么不在 USB 总线上

echo "=== [1] udev 规则有没有被动过 ==="
ls -l --time-style=long-iso /etc/udev/rules.d/

echo
echo "=== [2] restart_hardware.sh 有没有碰 USB（应该没有输出）==="
grep -nE 'usb|udev|modprobe|authorized|/sys/bus' \
  "${HOME}/r2_mapping/scripts/restart_hardware.sh" || echo "    （没有，确认没碰 USB）"

echo
echo "=== [3] USB 拓扑 + 端口状态 ==="
lsusb -t

echo
echo "=== [4] 每个 USB 设备节点 ==="
for d in /sys/bus/usb/devices/*/; do
  v="$(cat "$d/idVendor" 2>/dev/null)"
  p="$(cat "$d/idProduct" 2>/dev/null)"
  n="$(cat "$d/product" 2>/dev/null)"
  [ -n "$v" ] || continue
  echo "    $(basename "$d")  ${v}:${p}  ${n}"
done

echo
echo "=== [5] 5 口 Hub 每个端口上有没有设备 ==="
for h in /sys/bus/usb/devices/*/; do
  if [ -f "$h/maxchild" ] && [ -f "$h/idVendor" ]; then
    n="$(cat "$h/product" 2>/dev/null)"
    echo "  Hub $(basename "$h") ${n}  端口数 $(cat "$h/maxchild")"
    for i in $(seq 1 "$(cat "$h/maxchild")"); do
      if [ -d "${h}${i}-"* ] 2>/dev/null || ls -d "${h}${i}-"* >/dev/null 2>&1; then
        echo "      port $i : 有设备"
      fi
    done
  fi
done

echo
echo "=== [6] 有没有 usbguard / 授权限制 ==="
which usbguard 2>/dev/null || echo "    没有 usbguard"
ls /sys/bus/usb/devices/*/authorized 2>/dev/null | while read -r f; do
  echo "    $(basename "$(dirname "$f")") authorized=$(cat "$f")"
done

echo
echo "=== [7] 本次启动内核认到的 tty 设备 ==="
journalctl -k -b 2>/dev/null | grep -iE 'ttyUSB|ch34x|cp210|ftdi' | tail -10

echo
echo "=== [8] 电源/过流报警 ==="
journalctl -k -b 2>/dev/null | grep -iE 'over-current|overcurrent|power' | tail -5 \
  || echo "    没有过流记录"

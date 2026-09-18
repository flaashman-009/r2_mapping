#!/usr/bin/env bash
# 一键看车上 USB 外设的状态。
# 插一根线就跑一次，看哪一行变 [OK]。

echo "=============================================="
echo " 车载 USB 外设状态   $(date +%T)"
echo "=============================================="

check() {
  local name="$1" match="$2" dev="$3"
  if lsusb | grep -qi "$match"; then
    printf '  [OK]   %s\n' "$name"
  else
    printf '  [缺失] %s   (期望 USB ID: %s)\n' "$name" "$match"
  fi
  if [ -n "$dev" ]; then
    if [ -e "$dev" ]; then
      printf '         %s 存在\n' "$dev"
    else
      printf '         %s 不存在\n' "$dev"
    fi
  fi
}

check "底盘串口 (STM32)"  "1a86:7523" "/dev/myserial"
check "雷达 (LiDAR)"      "10c4:ea60" "/dev/ydlidar"
check "摄像头 (Astra)"    "2bc5:"     "/dev/video0"
check "语音模块"          "1a86:7522" "/dev/myspeech"
check "手柄接收器"        "0483:5750" ""

echo
echo "--- Hub 上实际挂着什么 ---"
lsusb -t

echo
echo "--- 所有串口设备 ---"
ls -l /dev/ttyUSB* /dev/ttyACM* 2>&1 | sed 's/^/  /'

#!/usr/bin/env bash
# 从臂 A1Z CAN 接口一键启用(HHS USB-CANFD 适配器 a8fa:8598,走 gs_usb)
# 用法: sudo bash "/home/tommyzihao/ROBOT ARM/setup_follower_can.sh"
set -e
BITRATE=1000000

echo "[1/4] 加载 gs_usb"
modprobe gs_usb 2>/dev/null || insmod "/home/tommyzihao/ROBOT ARM/gs_usb_module/gs_usb.ko"

echo "[2/4] 绑定 a8fa:8598 到 gs_usb"
echo "a8fa 8598" > /sys/bus/usb/drivers/gs_usb/new_id 2>/dev/null || true
sleep 1

echo "[3/4] 定位 gs_usb 生成的 CAN 接口(parentbus=usb)"
IFACE=""
for c in /sys/class/net/can*; do
  name=$(basename "$c")
  if readlink -f "$c/device" 2>/dev/null | grep -q "usb"; then IFACE=$name; break; fi
done
if [ -z "$IFACE" ]; then echo "  未找到 USB CAN 接口,请检查适配器是否插好"; exit 1; fi
echo "  找到: $IFACE"

echo "[4/4] 启用 $IFACE @ ${BITRATE}bps"
ip link set "$IFACE" down 2>/dev/null || true
ip link set "$IFACE" type can bitrate "$BITRATE"
ip link set "$IFACE" up
ip -d link show "$IFACE" | grep -oE "can state [A-Z-]*"
echo "完成。用以下命令扫描从臂电机:"
echo "  conda activate GALAXEA-lingbot"
echo "  python \"/home/tommyzihao/ROBOT ARM/GALAXEA-A1Z/tools/motor_diag.py\" --scan --channel $IFACE"

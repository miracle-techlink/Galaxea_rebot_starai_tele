#!/usr/bin/env bash
# 安装 peak_usb 内核模块(PEAK PCAN-USB -> SocketCAN,reBot B601-RS 用)。
# 很多 Jetson/Tegra 内核默认没编 CONFIG_CAN_PEAK_USB。本脚本优先从 mainline 取对应大版本
# 源码用当前内核重新编译;编译不了且预编译 .ko 的 vermagic 匹配时用预编译 .ko。
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
KREL="$(uname -r)"
DEST="/lib/modules/$KREL/kernel/drivers/net/can/usb"
KVER="${KVER:-v6.8}"   # 取源码的内核大版本 tag(与运行内核匹配, 如 6.8.x -> v6.8)

echo "[peak_usb] 目标内核: $KREL  源码 tag: $KVER"
if lsmod | grep -q '^peak_usb'; then echo "[peak_usb] 已加载, 跳过。"; exit 0; fi

KO="$HERE/peak_usb.ko"
if [ -d "/lib/modules/$KREL/build" ] && command -v make >/dev/null; then
  echo "[peak_usb] 用当前内核重新编译..."
  TMP="$(mktemp -d)"   # make M= 不吃带空格路径
  BASE="https://raw.githubusercontent.com/torvalds/linux/$KVER/drivers/net/can/usb/peak_usb"
  for f in pcan_usb_core.c pcan_usb_core.h pcan_usb.c pcan_usb_pro.c pcan_usb_pro.h pcan_usb_fd.c; do
    curl -sfL "$BASE/$f" -o "$TMP/$f"
  done
  printf 'obj-m := peak_usb.o\npeak_usb-y := pcan_usb_core.o pcan_usb.o pcan_usb_pro.o pcan_usb_fd.o\n' > "$TMP/Makefile"
  make -C "/lib/modules/$KREL/build" M="$TMP" modules && KO="$TMP/peak_usb.ko"
else
  echo "[peak_usb] 无内核头文件,尝试预编译 peak_usb.ko"
  if [ "$(modinfo "$KO" 2>/dev/null | awk '/vermagic/{print $2}')" != "$KREL" ]; then
    echo "[peak_usb] !! 预编译 .ko 与当前内核不匹配,需要内核头文件重新编译。中止。"; exit 1
  fi
fi

sudo mkdir -p "$DEST"
sudo cp "$KO" "$DEST/peak_usb.ko"
sudo depmod -a
echo peak_usb | sudo tee /etc/modules-load.d/peak_usb.conf >/dev/null   # 开机自动加载
sudo modprobe peak_usb
echo "[peak_usb] 完成。插上 PCAN-USB 会自动生成一个 canX 接口(本机实测为 can5)。"
echo "[peak_usb] 起总线: sudo ip link set canX up type can bitrate 1000000 restart-ms 100"

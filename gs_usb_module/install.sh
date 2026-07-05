#!/usr/bin/env bash
# 安装修补版 gs_usb 内核模块(HHS USB-CANFD a8fa:8598:EP OUT=1 + 加入 ID 表自动绑定)
# 优先用当前内核重新编译;编译不了且 vermagic 匹配时用预编译 .ko。
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
KREL="$(uname -r)"
DEST="/lib/modules/$KREL/kernel/drivers/net/can/usb"

echo "[gs_usb] 目标内核: $KREL"
KO="$HERE/gs_usb.ko"
if [ -d "/lib/modules/$KREL/build" ] && command -v make >/dev/null; then
  echo "[gs_usb] 用当前内核重新编译..."
  # make M= 不吃带空格路径,复制到无空格临时目录编译
  TMP="$(mktemp -d)"; cp "$HERE/gs_usb.c" "$HERE/Makefile" "$TMP/"
  make -C "/lib/modules/$KREL/build" M="$TMP" modules && KO="$TMP/gs_usb.ko"
else
  echo "[gs_usb] 无内核头文件,尝试用预编译 gs_usb.ko"
  if [ "$(modinfo "$KO" 2>/dev/null | awk '/vermagic/{print $2}')" != "$KREL" ]; then
    echo "[gs_usb] !! 预编译 .ko 与当前内核不匹配,需要内核头文件重新编译。中止。" ; exit 1
  fi
fi

sudo mkdir -p "$DEST"
sudo cp "$KO" "$DEST/gs_usb.ko"
sudo depmod -a
echo gs_usb | sudo tee /etc/modules-load.d/gs_usb.conf >/dev/null   # 开机自动加载
sudo modprobe gs_usb
echo "[gs_usb] 完成。插上 HHS 适配器会自动绑定并生成一个 canX 接口。"

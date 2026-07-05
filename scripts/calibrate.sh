#!/usr/bin/env bash
# 一键标定两臂(零位 + 各关节量程 + 夹爪开合)。首次或重摆位后运行。
set -e
CONDA_ENV="${CONDA_ENV:-lerobot}"
LEADER_PORT="${LEADER_PORT:-/dev/ttyCH341USB0}"
CONDA_BASE="$(conda info --base 2>/dev/null || echo /home/tommyzihao/miniconda3)"
source "$CONDA_BASE/etc/profile.d/conda.sh"; conda activate "$CONDA_ENV"

CAN_IF="${CAN_IF:-}"
if [ -z "$CAN_IF" ]; then
  for c in /sys/class/net/can*; do n=$(basename "$c")
    readlink -f "$c/device" 2>/dev/null | grep -q usb && CAN_IF="$n" && break; done
fi
CAN_IF="${CAN_IF:-can4}"
sudo modprobe gs_usb 2>/dev/null || true
sudo ip link set "$CAN_IF" up type can bitrate 1000000 2>/dev/null || true

echo "==== ① 从臂标定 (CAN=$CAN_IF) ===="
echo "提示: 柔顺悬浮后 → 摆零位(各关节居中)按Enter → 扫每关节到两端(含夹爪开合到底)按Enter"
lerobot-calibrate --robot.type=galaxea_a1z_follower --robot.can_channel="$CAN_IF" --robot.id=follower1

echo "==== ② 主臂标定 ($LEADER_PORT) ===="
echo "提示: 摆到与从臂对应的零位按Enter → 扫每关节到两端按Enter"
lerobot-calibrate --teleop.type=starai_violin_leader --teleop.port="$LEADER_PORT" --teleop.id=leader1

echo "标定完成 ✅  运行 bash scripts/teleop.sh 开始遥操作"

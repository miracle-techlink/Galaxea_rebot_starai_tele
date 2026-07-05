#!/usr/bin/env bash
# 一键遥操作:StarAI Violin(主) -> Galaxea A1Z(从,含夹爪)
# 需先跑过 setup.sh 和 calibrate.sh。
set -e
CONDA_ENV="${CONDA_ENV:-lerobot}"
LEADER_PORT="${LEADER_PORT:-/dev/ttyCH341USB0}"

CONDA_BASE="$(conda info --base 2>/dev/null || echo /home/tommyzihao/miniconda3)"
source "$CONDA_BASE/etc/profile.d/conda.sh"; conda activate "$CONDA_ENV"

# 自动探测 gs_usb(USB-CANFD)生成的 CAN 接口
CAN_IF="${CAN_IF:-}"
if [ -z "$CAN_IF" ]; then
  for c in /sys/class/net/can*; do n=$(basename "$c")
    readlink -f "$c/device" 2>/dev/null | grep -q usb && CAN_IF="$n" && break; done
fi
CAN_IF="${CAN_IF:-can4}"
echo "[teleop] CAN=$CAN_IF  主臂=$LEADER_PORT"
sudo modprobe gs_usb 2>/dev/null || true
sudo ip link set "$CAN_IF" up type can bitrate 1000000 2>/dev/null || true

# 方向/安全参数均为默认(已内置);如需覆盖用 --robot.xxx=
exec lerobot-teleoperate \
    --robot.type=galaxea_a1z_follower --robot.can_channel="$CAN_IF" --robot.id=follower1 \
    --robot.joint_sign='[-1,1,1,1,-1,-1]' \
    --teleop.type=starai_violin_leader --teleop.port="$LEADER_PORT" --teleop.id=leader1 \
    --fps=30 "$@"

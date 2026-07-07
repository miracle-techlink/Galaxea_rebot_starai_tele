#!/usr/bin/env bash
# 一键双臂遥操作:StarAI Violin(主) -> Galaxea A1Z + Seeed reBot B601-RS(从)同步镜像。
# 需先跑过 setup.sh、两臂各自 calibrate、以及 reBot 行程扫描(见 README)。
# ⚠️ 安全:两臂各自固定牢、各留 1 米净空、手放急停;起步先把主臂摆到两从臂当前接近的姿态。
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
CONDA_ENV="${CONDA_ENV:-lerobot}"
LEADER_PORT="${LEADER_PORT:-/dev/ttyCH341USB0}"
GALAXEA_CAN="${GALAXEA_CAN:-can0}"   # gs_usb (HHS USB-CANFD)
REBOT_CAN="${REBOT_CAN:-can5}"       # PCAN-USB
GALAXEA_FLIP="${GALAXEA_FLIP:-1,5,6}"

CONDA_BASE="$(conda info --base 2>/dev/null || echo /home/tommyzihao/miniconda3)"
source "$CONDA_BASE/etc/profile.d/conda.sh"; conda activate "$CONDA_ENV"

echo "[both] Galaxea CAN=$GALAXEA_CAN  reBot CAN=$REBOT_CAN  主臂=$LEADER_PORT  galaxea_flip=$GALAXEA_FLIP"
sudo modprobe gs_usb 2>/dev/null || true
sudo modprobe peak_usb 2>/dev/null || true
sudo ip link set "$GALAXEA_CAN" up type can bitrate 1000000 restart-ms 100 2>/dev/null || true
sudo ip link set "$REBOT_CAN"  up type can bitrate 1000000 restart-ms 100 2>/dev/null || true

exec python "$HERE/teleop_starai_to_both.py" --arms both --go \
    --leader-port "$LEADER_PORT" \
    --galaxea-can "$GALAXEA_CAN" --galaxea-flip "$GALAXEA_FLIP" --galaxea-max-vel-deg-s 90 --galaxea-max-step-deg 3 \
    --rebot-can "$REBOT_CAN" --match-range --no-limit --grip-ratio-min 0.62 \
    "$@"

#!/usr/bin/env bash
# 官方 lerobot-teleoperate 驱动 dual_follower(一支 StarAI 主臂 → 同驱 Galaxea + reBot),
# 双腕 Orbbec 相机进 rerun。Ctrl-C 停(两臂自动归位:reBot 回启动 home,Galaxea 回零)。
#
# 用法:  PY=/path/to/lerobot-env/python  bash scripts/teleop_dual.sh  [额外 --key=val ...]
#   环境变量(可选覆盖): PY / LEADER_PORT / GALAXEA_CAM / REBOT_CAM / GALAXEA_FLIP / NO_CAM=1
#
# 前提: 已 bash setup.sh(装 dual_follower 插件) + 两臂各自标定 + CAN 起(setup_follower_can.sh)。
set -e
PY="${PY:-python}"                                   # 指向装了 lerobot 的 conda env python
LEADER_PORT="${LEADER_PORT:-/dev/ttyCH341USB0}"
GALAXEA_CAM="${GALAXEA_CAM:-CV2856D0006R}"
REBOT_CAM="${REBOT_CAM:-CV275610002L}"
GALAXEA_FLIP="${GALAXEA_FLIP:-1,5,6}"

# rerun viewer 需在 PATH 里找到(与 env python 同目录)
BIN_DIR="$(dirname "$("$PY" -c 'import sys; print(sys.executable)')")"
export PATH="$BIN_DIR:$PATH"

CAM_ARG=()
if [ "${NO_CAM:-0}" != "1" ]; then
  # USB2 链路务必 mjpg(未压缩 RGB 会撑爆带宽 → 取帧 8s 超时崩溃)
  CAMS="{ galaxea_wrist: {type: orbbec, serial_number_or_name: ${GALAXEA_CAM}, fps: 30, width: 640, height: 480, color_format: mjpg}, rebot_wrist: {type: orbbec, serial_number_or_name: ${REBOT_CAM}, fps: 30, width: 640, height: 480, color_format: mjpg} }"
  CAM_ARG=(--robot.cameras="${CAMS}")
fi

exec lerobot-teleoperate \
  --robot.type=dual_follower --robot.id=dual1 \
  --robot.galaxea_flip="${GALAXEA_FLIP}" \
  "${CAM_ARG[@]}" \
  --teleop.type=starai_violin_leader --teleop.port="${LEADER_PORT}" --teleop.id=leader1 \
  --fps=30 --display_data=true "$@"

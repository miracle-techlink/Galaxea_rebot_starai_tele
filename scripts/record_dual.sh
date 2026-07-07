#!/usr/bin/env bash
# 用官方 lerobot-record 采集双臂遥操作数据集(dual_follower + StarAI 主臂 + 双腕相机)。
# 记录: observation.state = {galaxea_*, rebot_*} 关节 + observation.images.{galaxea_wrist,rebot_wrist}
#        action = 主臂 7 维(joint_1..6.pos + gripper.pos)。Ctrl-C/定时结束后两臂自动归位。
#
# 用法:  PY=/path/to/env/python REPO_ID=me/dual_pick TASK="pick the cube" \
#           bash scripts/record_dual.sh [--dataset.num_episodes=20 ...]
#   环境变量: PY / LEADER_PORT / GALAXEA_CAM / REBOT_CAM / GALAXEA_FLIP / REPO_ID / TASK / EPISODES / PUSH
set -e
PY="${PY:-python}"
LEADER_PORT="${LEADER_PORT:-/dev/ttyCH341USB0}"
GALAXEA_CAM="${GALAXEA_CAM:-CV2856D0006R}"
REBOT_CAM="${REBOT_CAM:-CV275610002L}"
GALAXEA_FLIP="${GALAXEA_FLIP:-1,5,6}"
REPO_ID="${REPO_ID:?请设 REPO_ID=你的用户名/数据集名}"
TASK="${TASK:?请设 TASK=\"任务自然语言描述\"}"
EPISODES="${EPISODES:-10}"
PUSH="${PUSH:-false}"

BIN_DIR="$(dirname "$("$PY" -c 'import sys; print(sys.executable)')")"
export PATH="$BIN_DIR:$PATH"

CAMS="{ galaxea_wrist: {type: orbbec, serial_number_or_name: ${GALAXEA_CAM}, fps: 30, width: 640, height: 480, color_format: mjpg}, rebot_wrist: {type: orbbec, serial_number_or_name: ${REBOT_CAM}, fps: 30, width: 640, height: 480, color_format: mjpg} }"

exec lerobot-record \
  --robot.type=dual_follower --robot.id=dual1 \
  --robot.galaxea_flip="${GALAXEA_FLIP}" \
  --robot.cameras="${CAMS}" \
  --teleop.type=starai_violin_leader --teleop.port="${LEADER_PORT}" --teleop.id=leader1 \
  --fps=30 --display_data=true \
  --dataset.repo_id="${REPO_ID}" \
  --dataset.single_task="${TASK}" \
  --dataset.num_episodes="${EPISODES}" \
  --dataset.push_to_hub="${PUSH}" "$@"

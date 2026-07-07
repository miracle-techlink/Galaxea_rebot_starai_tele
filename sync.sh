#!/usr/bin/env bash
# 把 ~/ROBOT ARM 的工作副本 + lerobot 插件同步进本 repo(并做路径可移植化),供提交上传。
# 用法: bash sync.sh   然后  git add -A && git commit -m "..." && bash push.sh
set -e
D="$(cd "$(dirname "$0")" && pwd)"
RA="${RA:-/home/tommyzihao/ROBOT ARM}"
LB="${LB:-/home/tommyzihao/lingbot/lerobot/src/lerobot}"

echo "[sync] scripts + 配置"
cp "$RA/teleop_starai_to_rebot.py" "$RA/teleop_starai_to_both.py" "$RA/teleop_starai_to_a1z.py" "$D/scripts/"
cp "$RA/rebot_follower_range.json" "$D/scripts/"
cp "$RA/scan_leader_starai.py" "$RA/setup_follower_can.sh" "$D/scripts/" 2>/dev/null || true

echo "[sync] lerobot 插件"
cp "$LB/robots/galaxea_a1z_follower/"*.py "$D/lerobot_plugins/robots/galaxea_a1z_follower/"
cp "$LB/teleoperators/starai_violin_leader/"*.py "$D/lerobot_plugins/teleoperators/starai_violin_leader/"
mkdir -p "$D/lerobot_plugins/cameras/orbbec"
cp "$LB/cameras/orbbec/"*.py "$D/lerobot_plugins/cameras/orbbec/"

echo "[sync] docs"
cp "$RA/TELEOP_LEROBOT.md" "$RA/SETUP_LOG.md" "$D/docs/" 2>/dev/null || true

echo "[sync] 路径可移植化(去掉硬编码绝对路径)"
python - "$D" <<'PY'
import sys, io, os
D=sys.argv[1]
for fn in ["teleop_starai_to_rebot.py","teleop_starai_to_both.py"]:
    p=os.path.join(D,"scripts",fn); s=io.open(p,encoding="utf-8").read()
    s=s.replace('RANGE_FILE_DEFAULT = "/home/tommyzihao/ROBOT ARM/rebot_follower_range.json"',
                'RANGE_FILE_DEFAULT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "rebot_follower_range.json")')
    s=s.replace('RANGE_FILE = "/home/tommyzihao/ROBOT ARM/rebot_follower_range.json"',
                'RANGE_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "rebot_follower_range.json")')
    import re
    s=re.sub(r'\(f?"/home/tommyzihao/\.cache/huggingface/lerobot/calibration/teleoperators/"\s*\n\s*f"starai_violin_leader/\{(\w+)\.leader_id\}\.json"\)',
             r'os.path.expanduser(f"~/.cache/huggingface/lerobot/calibration/teleoperators/starai_violin_leader/{\1.leader_id}.json")', s)
    io.open(p,"w",encoding="utf-8").write(s)
    import ast; ast.parse(s); print("  ok", fn)
PY
echo "[sync] 完成。接着: git -C \"$D\" add -A && git -C \"$D\" commit -m '...' && bash \"$D/push.sh\""

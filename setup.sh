#!/usr/bin/env bash
# 一键安装:两臂 SDK 依赖 + lerobot 插件 + gs_usb 驱动。
# 前提:已有一个装好 lerobot 的 conda 环境(默认名 lerobot)。
#
# 用法:
#   bash setup.sh
#   CONDA_ENV=lerobot LEROBOT_SRC=/home/tommyzihao/lingbot/lerobot bash setup.sh
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
CONDA_ENV="${CONDA_ENV:-lerobot}"
LEROBOT_SRC="${LEROBOT_SRC:-/home/tommyzihao/lingbot/lerobot}"

CONDA_BASE="$(conda info --base 2>/dev/null || echo /home/tommyzihao/miniconda3)"
source "$CONDA_BASE/etc/profile.d/conda.sh"
conda activate "$CONDA_ENV"
PY="$(python -c 'import sys;print(sys.executable)')"
echo "[setup] env=$CONDA_ENV  python=$PY  lerobot=$LEROBOT_SRC"

echo "[setup] 1/4 装 Python 依赖 (python-can / pyserial)"
$PY -m pip install "python-can>=4.0" pyserial >/dev/null

echo "[setup] 2/4 装机械臂 SDK"
$PY -m pip install -e "$HERE/third_party/GALAXEA-A1Z" --no-deps >/dev/null   # a1z 从臂 SDK
conda install -y -n "$CONDA_ENV" -c conda-forge pinocchio >/dev/null 2>&1     # a1z 依赖
# 主臂 uservo:.pth 加入 site-packages
SP="$($PY -c 'import site;print(site.getsitepackages()[0])')"
echo "$HERE/third_party/fashionstar-uart-servo-python/src" > "$SP/uservo_src.pth"

echo "[setup] 3/4 装 lerobot 插件"
LEROBOT_SRC="$LEROBOT_SRC" bash "$HERE/lerobot_plugins/install.sh"

echo "[setup] 4/4 装 gs_usb 驱动(需 sudo)"
bash "$HERE/gs_usb_module/install.sh"

echo
echo "[setup] 完成 ✅  验证:"
$PY - <<'EOF'
for m in ["lerobot","can","serial","pinocchio","a1z","uservo"]:
    try: __import__(m); print("  OK  ", m)
    except Exception as e: print("  FAIL", m, e)
EOF
echo "接着: bash scripts/calibrate.sh  然后  bash scripts/teleop.sh"

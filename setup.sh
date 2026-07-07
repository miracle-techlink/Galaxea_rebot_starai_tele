#!/usr/bin/env bash
# 一键安装:双臂遥操作(StarAI 主 -> Galaxea A1Z + Seeed reBot B601-RS 从)+ 可选 Orbbec 深度相机。
# 前提:已有一个装好 lerobot 的 conda 环境(默认名 lerobot)。
#
# 用法:
#   bash setup.sh                 # 全装(Galaxea + reBot + 可选相机)
#   WITH_REBOT=0 bash setup.sh    # 只装 Galaxea
#   WITH_ORBBEC=0 bash setup.sh   # 不装相机
#   CONDA_ENV=lerobot LEROBOT_SRC=/home/tommyzihao/lingbot/lerobot bash setup.sh
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
CONDA_ENV="${CONDA_ENV:-lerobot}"
LEROBOT_SRC="${LEROBOT_SRC:-/home/tommyzihao/lingbot/lerobot}"
WITH_REBOT="${WITH_REBOT:-1}"
WITH_ORBBEC="${WITH_ORBBEC:-1}"
PIP_INDEX="${PIP_INDEX:-https://pypi.org/simple}"   # 国内镜像常拉不动大 wheel,默认官方源

CONDA_BASE="$(conda info --base 2>/dev/null || echo /home/tommyzihao/miniconda3)"
source "$CONDA_BASE/etc/profile.d/conda.sh"; conda activate "$CONDA_ENV"
PY="$(python -c 'import sys;print(sys.executable)')"
echo "[setup] env=$CONDA_ENV  python=$PY  lerobot=$LEROBOT_SRC  reBot=$WITH_REBOT  orbbec=$WITH_ORBBEC"

echo "[setup] Python 依赖 (python-can / pyserial)"
$PY -m pip install "python-can>=4.0" pyserial >/dev/null

echo "[setup] Galaxea + StarAI: SDK + lerobot 插件"
$PY -m pip install -e "$HERE/third_party/GALAXEA-A1Z" --no-deps >/dev/null      # a1z 从臂 SDK
conda install -y -n "$CONDA_ENV" -c conda-forge pinocchio >/dev/null 2>&1        # a1z 依赖
SP="$($PY -c 'import site;print(site.getsitepackages()[0])')"
echo "$HERE/third_party/fashionstar-uart-servo-python/src" > "$SP/uservo_src.pth" # 主臂 uservo
LEROBOT_SRC="$LEROBOT_SRC" bash "$HERE/lerobot_plugins/install.sh"
echo "[setup] Galaxea CAN 驱动 (gs_usb, 需 sudo)"
bash "$HERE/gs_usb_module/install.sh"

if [ "$WITH_REBOT" = "1" ]; then
  echo "[setup] reBot B601-RS: motorbridge + 官方插件包(RobStride 从臂 + 102 主臂)"
  $PY -m pip install -i "$PIP_INDEX" "motorbridge>=0.3.2,<0.4.0" "motorbridge-smart-servo>=0.0.4,<0.1.0" >/dev/null
  $PY -m pip install --no-deps -i "$PIP_INDEX" lerobot-robot-seeed-b601 lerobot-teleoperator-rebot-arm-102 >/dev/null
  echo "[setup] reBot CAN 驱动 (peak_usb / PCAN-USB, 需 sudo)"
  bash "$HERE/peak_usb_module/install.sh" || echo "[setup] !! peak_usb 装失败(内核头文件?),见 peak_usb_module/README.md"
fi

if [ "$WITH_ORBBEC" = "1" ]; then
  echo "[setup] Orbbec 深度相机: pyorbbecsdk2 + 插件 + udev"
  if $PY -m pip install -i "$PIP_INDEX" pyorbbecsdk2 >/dev/null 2>&1; then
    LEROBOT_SRC="$LEROBOT_SRC" bash "$HERE/lerobot_plugins/install_orbbec.sh"
    UDEV="$($PY -c 'import pyorbbecsdk,os;print(os.path.dirname(pyorbbecsdk.__file__))' 2>/dev/null)/shared/install_udev_rules.sh"
    [ -f "$UDEV" ] && sudo sh "$UDEV" || echo "[setup] 未找到 udev 脚本, 手动装 orbbec udev 规则"
  else
    echo "[setup] !! pyorbbecsdk2 装失败(网络?), 跳过相机。"
  fi
fi

echo
echo "[setup] 完成 ✅  验证:"
$PY - <<'EOF'
mods = ["lerobot","can","serial","pinocchio","a1z","uservo","motorbridge","pyorbbecsdk"]
for m in mods:
    try: __import__(m); print("  OK  ", m)
    except Exception: print("  --  ", m, "(未装/可选)")
EOF
echo "接着: 标定(见 README) -> bash scripts/teleop.sh (Galaxea) / teleop_both.sh (双臂)"

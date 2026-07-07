#!/usr/bin/env bash
# 把两个自定义插件装进 lerobot 源码树,并在 __init__.py 里注册(触发 register_subclass)。
# 用法: LEROBOT_SRC=/path/to/lerobot bash install.sh   (默认 /home/tommyzihao/lingbot/lerobot)
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
LEROBOT_SRC="${LEROBOT_SRC:-/home/tommyzihao/lingbot/lerobot}"
LB="$LEROBOT_SRC/src/lerobot"
[ -d "$LB" ] || { echo "!! 找不到 lerobot 源码: $LB (设 LEROBOT_SRC)"; exit 1; }

echo "[plugins] 复制插件到 $LB"
cp -r "$HERE/robots/galaxea_a1z_follower" "$LB/robots/"
cp -r "$HERE/robots/dual_follower" "$LB/robots/"
cp -r "$HERE/teleoperators/starai_violin_leader" "$LB/teleoperators/"

# 幂等地在 __init__.py 追加注册导入
add_import() {
  local f="$1" line="$2"
  grep -qF "$line" "$f" || echo "$line" >> "$f"
}
add_import "$LB/robots/__init__.py" "from . import galaxea_a1z_follower  # noqa: F401"
# dual_follower 依赖 galaxea_a1z_follower(树内)+ lerobot_robot_seeed_b601(pip, 懒加载)
add_import "$LB/robots/__init__.py" "from . import dual_follower  # noqa: F401"
add_import "$LB/teleoperators/__init__.py" "from . import starai_violin_leader  # noqa: F401"
echo "[plugins] 完成。type: galaxea_a1z_follower / dual_follower / starai_violin_leader 已注册。"

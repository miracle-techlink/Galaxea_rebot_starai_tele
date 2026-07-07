#!/usr/bin/env python
"""StarAI Violin (leader) -> Galaxea A1Z (follower) 关节空间遥操作桥接(直通版).

映射逻辑现在在 LeRobot 类内部:主臂标定后输出"零位为0"的角度,从臂 send_action 做
`target = zero_offset + sign*scale*deg2rad(leader_deg)` + 限速。本脚本只是 get_action->
send_action 的直通,外加一个安全的 --dry-run(从臂不动,打印映射目标供核对方向)。

前提:先各自标定一次(零位对齐):
    lerobot-calibrate --teleop.type=starai_violin_leader --teleop.port=/dev/ttyCH341USB0 --teleop.id=leader1
    lerobot-calibrate --robot.type=galaxea_a1z_follower  --robot.can_channel=can4 --robot.id=follower1

环境: conda activate lerobot
用法:
    python "/home/tommyzihao/ROBOT ARM/teleop_starai_to_a1z.py" --dry-run        # 从臂不动, 验证方向
    python "/home/tommyzihao/ROBOT ARM/teleop_starai_to_a1z.py" --flip 2,4        # 正式跟随
Ctrl-C 安全停止。
"""
import argparse
import signal
import sys
import time

import numpy as np

from lerobot.robots.galaxea_a1z_follower import GalaxeaA1ZFollower, GalaxeaA1ZFollowerConfig
from lerobot.teleoperators.starai_violin_leader import StaraiViolinLeader, StaraiViolinLeaderConfig

NUM_JOINTS = 6
ARM_KEYS = [f"joint_{i + 1}.pos" for i in range(NUM_JOINTS)]


def main():
    ap = argparse.ArgumentParser(description="StarAI Violin -> Galaxea A1Z teleop (直通)")
    ap.add_argument("--leader-port", default="/dev/ttyCH341USB0")
    ap.add_argument("--can", default="can4")
    ap.add_argument("--freq", type=float, default=50.0)
    ap.add_argument("--flip", default="", help="需翻转方向的关节, 如 '2,4' (1-6)")
    ap.add_argument("--scale", type=float, default=1.0)
    ap.add_argument("--max-step-deg", type=float, default=1.0)
    ap.add_argument("--dry-run", action="store_true", help="从臂不动, 只打印映射目标")
    args = ap.parse_args()

    sign = [1, 1, 1, 1, 1, 1]
    for tok in args.flip.split(","):
        if tok.strip():
            sign[int(tok) - 1] = -1

    print("=" * 68)
    print(f"  StarAI Violin -> Galaxea A1Z  ({'DRY-RUN' if args.dry_run else '正式跟随'})")
    print(f"  sign={sign}  scale={args.scale}  freq={args.freq}Hz  step={args.max_step_deg}deg")
    print("=" * 68)

    leader = StaraiViolinLeader(StaraiViolinLeaderConfig(port=args.leader_port))
    follower = GalaxeaA1ZFollower(GalaxeaA1ZFollowerConfig(
        can_channel=args.can, joint_sign=sign, scale=args.scale,
        max_step_rad=np.deg2rad(args.max_step_deg),
    ))

    signal.signal(signal.SIGINT, signal.default_int_handler)
    leader.connect()
    follower.connect()
    print("开始。Ctrl-C 停止。\n")

    dt = 1.0 / args.freq
    try:
        while True:
            t0 = time.perf_counter()
            action = leader.get_action()
            if args.dry_run:
                tgt = follower._resolve_target(action)
                cur = np.array([follower.get_observation()[k] for k in ARM_KEYS])
                print("  主臂(deg): " + " ".join(f"{action[k]:+6.1f}" for k in ARM_KEYS) +
                      "  从臂目标(deg): " + " ".join(f"{np.degrees(t):+6.1f}" for t in tgt) +
                      "  当前: " + " ".join(f"{np.degrees(c):+6.1f}" for c in cur), end="\r")
            else:
                follower.send_action(action)
            sleep = dt - (time.perf_counter() - t0)
            if sleep > 0:
                time.sleep(sleep)
    except KeyboardInterrupt:
        print("\n停止...")
    finally:
        follower.disconnect()
        leader.disconnect()
        print("已安全断开。")


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python
"""一个 StarAI Violin 主臂 → 同时驱动 Galaxea A1Z + Seeed reBot B601-RS 双从臂(镜像同动作).

读一次 leader.get_action() → 分别喂两个 follower:
  - Galaxea A1Z:内部 teleop_input="leader_deg" 映射(直接把 leader 的 joint_1..6.pos + gripper.pos 交给它),
    方向/幅度由 --galaxea-flip / --galaxea-scale 控制,CAN=can0(gs_usb)。
  - reBot B601-RS(RobStride):本脚本做 joint_1..6→shoulder_pan.. 映射 + 夹爪直驱 7 号电机,
    行程按 config 限位匹配(--match-range),CAN=can5(PCAN)。夹爪参数与 teleop_starai_to_rebot.py 一致。

--arms 选择驱动哪条:galaxea / rebot / both(默认 both)。默认 dry-run(不发指令);--go 才真正驱动。
退出:reBot 平滑回零;Galaxea 靠自身 return_to_zero_on_exit 回零。

先各自标定过(见 TELEOP_LEROBOT.md §2/§8)。CAN 先起:
  sudo ip link set can0 up type can bitrate 1000000 restart-ms 100   # Galaxea gs_usb
  sudo ip link set can5 up type can bitrate 1000000 restart-ms 100   # reBot PCAN

用法:
  # 先单验 Galaxea(reBot 不动),慢扳主臂看 Galaxea 方向,反的记进 --galaxea-flip
  python teleop_starai_to_both.py --arms galaxea --go --galaxea-max-step-deg 1 --freq 20
  # 双臂一起(方向都验过后)
  python teleop_starai_to_both.py --arms both --go --match-range --no-limit \
      --galaxea-flip 2,4 --grip-ratio-min 0.62
⚠️ 安全:两臂都固定牢、各自 1 米净空、手放急停。
"""
import argparse
import json
import math
import os
import signal
import sys
import time

import numpy as np

LEADER_KEYS = [f"joint_{i + 1}.pos" for i in range(6)]
REBOT_ARM_MOTORS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_yaw", "wrist_roll"]
GRIPPER_MOTOR = "gripper"
RANGE_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "rebot_follower_range.json")


def parse_flip(s):
    sign = [1, 1, 1, 1, 1, 1]
    for tok in s.split(","):
        if tok.strip():
            sign[int(tok) - 1] = -1
    return sign


def main():
    ap = argparse.ArgumentParser(description="StarAI -> Galaxea A1Z + reBot B601-RS 双臂遥操作")
    ap.add_argument("--arms", choices=["galaxea", "rebot", "both"], default="both")
    ap.add_argument("--leader-port", default="/dev/ttyCH341USB0")
    ap.add_argument("--leader-id", default="leader1")
    ap.add_argument("--freq", type=float, default=30.0)
    ap.add_argument("--go", action="store_true", help="真正驱动(默认 dry-run 只打印)")
    ap.add_argument("--no-return", action="store_true", help="退出不回零")
    # Galaxea
    ap.add_argument("--galaxea-can", default="can0")
    ap.add_argument("--galaxea-id", default="follower1")
    ap.add_argument("--galaxea-flip", default="2,4", help="Galaxea 反向关节(默认暂定 2,4)")
    ap.add_argument("--galaxea-scale", type=float, default=1.0)
    ap.add_argument("--galaxea-max-step-deg", type=float, default=3.0, help="Galaxea 每步步长上限(度), 越大越快")
    ap.add_argument("--galaxea-max-vel-deg-s", type=float, default=90.0,
                    help="Galaxea 关节速度上限(度/秒), 默认50偏慢, 放大更跟手(别过大防甩臂)")
    # reBot
    ap.add_argument("--rebot-can", default="can5")
    ap.add_argument("--rebot-id", default="follower1")
    ap.add_argument("--rebot-flip", default="", help="reBot 反向关节(单验已确认全对, 一般空)")
    ap.add_argument("--match-range", action="store_true", help="reBot 行程按 config 限位匹配, 零位对齐")
    ap.add_argument("--rebot-scale", type=float, default=1.0)
    ap.add_argument("--max-step-deg", type=float, default=3.0, help="reBot 每步限速(度)")
    ap.add_argument("--no-limit", action="store_true", help="reBot 不限速")
    # reBot 夹爪(直驱 7 号)
    ap.add_argument("--grip-kp", type=float, default=9.0)
    ap.add_argument("--grip-kd", type=float, default=0.3)
    ap.add_argument("--grip-max-step-deg", type=float, default=25.0)
    ap.add_argument("--grip-clamp-deg", type=float, default=25.0)
    ap.add_argument("--grip-ratio-min", type=float, default=0.62)
    ap.add_argument("--grip-ratio-max", type=float, default=1.0)
    ap.add_argument("--no-grip", action="store_true")
    args = ap.parse_args()

    use_gx = args.arms in ("galaxea", "both")
    use_rb = args.arms in ("rebot", "both")

    from lerobot.teleoperators.starai_violin_leader import StaraiViolinLeader, StaraiViolinLeaderConfig
    from lerobot.utils.import_utils import register_third_party_plugins
    register_third_party_plugins()

    leader = StaraiViolinLeader(StaraiViolinLeaderConfig(port=args.leader_port, id=args.leader_id))

    # ---------- Galaxea ----------
    gx = None
    if use_gx:
        from lerobot.robots.galaxea_a1z_follower import GalaxeaA1ZFollower, GalaxeaA1ZFollowerConfig
        gx = GalaxeaA1ZFollower(GalaxeaA1ZFollowerConfig(
            can_channel=args.galaxea_can, id=args.galaxea_id,
            joint_sign=parse_flip(args.galaxea_flip), scale=args.galaxea_scale,
            max_step_rad=math.radians(args.galaxea_max_step_deg),
            max_joint_vel_deg_s=args.galaxea_max_vel_deg_s,
            return_to_zero_on_exit=(not args.no_return),
        ))

    # ---------- reBot ----------
    rb = None
    rb_sign = parse_flip(args.rebot_flip)
    rb_gain = [args.rebot_scale] * 6
    grip_close = grip_open = grip_close_eff = None
    if use_rb:
        from lerobot_robot_seeed_b601.seeed_b601_rs_follower import SeeedB601RSFollower
        from lerobot_robot_seeed_b601.config_seeed_b601_rs_follower import SeeedB601RSFollowerConfig
        rb_cfg = SeeedB601RSFollowerConfig(
            port=args.rebot_can, can_adapter="socketcan", id=args.rebot_id,
            max_relative_target=(None if args.no_limit else args.max_step_deg),
        )
        rb = SeeedB601RSFollower(rb_cfg)
        if args.match_range:
            lp = os.path.expanduser(
                f"~/.cache/huggingface/lerobot/calibration/teleoperators/starai_violin_leader/{args.leader_id}.json")
            lc = json.load(open(lp))
            lmin, lmax = lc["range_min_deg"], lc["range_max_deg"]
            for i, m in enumerate(REBOT_ARM_MOTORS):
                lspan = abs(lmax[i] - lmin[i])
                flo, fhi = rb_cfg.joint_limits[m]
                rb_gain[i] = args.rebot_scale * (abs(fhi - flo) / lspan if lspan > 1e-6 else 1.0)
        swept = json.load(open(RANGE_FILE)) if os.path.exists(RANGE_FILE) else {}
        if GRIPPER_MOTOR in swept and abs(swept[GRIPPER_MOTOR][1] - swept[GRIPPER_MOTOR][0]) >= 5.0:
            grip_close, grip_open = swept[GRIPPER_MOTOR]
        else:
            grip_close, grip_open = rb_cfg.joint_limits[GRIPPER_MOTOR]
        close_dir = -1.0 if grip_close <= grip_open else 1.0
        grip_close_eff = grip_close + close_dir * args.grip_clamp_deg
    grip_follow = use_rb and args.go and not args.no_grip

    print("=" * 80)
    print(f"  StarAI -> [{'Galaxea' if use_gx else ''}{'+' if use_gx and use_rb else ''}{'reBot' if use_rb else ''}]"
          f"   [{'LIVE' if args.go else 'DRY-RUN'}]  freq={args.freq}Hz")
    if use_gx:
        print(f"  Galaxea: can={args.galaxea_can} flip={args.galaxea_flip} scale={args.galaxea_scale} step={args.galaxea_max_step_deg}deg vel={args.galaxea_max_vel_deg_s}deg/s")
    if use_rb:
        print(f"  reBot:   can={args.rebot_can} match_range={args.match_range} max_step={'无限速' if args.no_limit else args.max_step_deg}")
        if args.match_range:
            print("           增益: " + " ".join(f"{rb_gain[i]:.2f}" for i in range(6)))
        print(f"           夹爪跟随={grip_follow} ratio[{args.grip_ratio_min},{args.grip_ratio_max}]->[{grip_close_eff:.0f},{grip_open:.0f}]deg")
    print("=" * 80)

    signal.signal(signal.SIGINT, signal.default_int_handler)
    leader.connect()
    if gx is not None:
        gx.connect()
    if rb is not None:
        rb.connect()
        if not args.go:
            try:
                rb.disable_torque()
            except Exception:
                pass

    # reBot 映射 + 夹爪
    def rebot_arm(la):
        return {f"{m}.pos": rb_sign[i] * rb_gain[i] * float(la[LEADER_KEYS[i]]) for i, m in enumerate(REBOT_ARM_MOTORS)}

    grip_set = [None]

    def drive_gripper(target_deg):
        if grip_set[0] is None:
            grip_set[0] = target_deg
        d = max(-args.grip_max_step_deg, min(args.grip_max_step_deg, target_deg - grip_set[0]))
        grip_set[0] += d
        gm = rb.motors.get(GRIPPER_MOTOR)
        if gm is not None:
            gm.send_mit(math.radians(grip_set[0]), 0.0, args.grip_kp, args.grip_kd, 0.0)

    last_rb = {f"{m}.pos": 0.0 for m in REBOT_ARM_MOTORS}
    dt = 1.0 / args.freq
    try:
        while True:
            t0 = time.perf_counter()
            la = leader.get_action()
            if args.go:
                if gx is not None:
                    gx.send_action(la)                       # Galaxea 内部做 leader_deg 映射
                if rb is not None:
                    arm = rebot_arm(la)
                    last_rb = arm
                    rb.send_action(arm)
                    if grip_follow:
                        rr = float(la.get("gripper.pos", 0.0))
                        denom = max(args.grip_ratio_max - args.grip_ratio_min, 1e-3)
                        ratio = min(1.0, max(0.0, (rr - args.grip_ratio_min) / denom))
                        drive_gripper(grip_close_eff + ratio * (grip_open - grip_close_eff))
            else:
                parts = []
                if gx is not None:
                    tg = gx._resolve_target(la)
                    parts.append("GX目标(deg): " + " ".join(f"{np.degrees(t):+6.1f}" for t in tg))
                if rb is not None:
                    arm = rebot_arm(la)
                    parts.append("RB目标(deg): " + " ".join(f"{arm[f'{m}.pos']:+6.1f}" for m in REBOT_ARM_MOTORS))
                print("  " + "  |  ".join(parts), end="\r")
            slp = dt - (time.perf_counter() - t0)
            if slp > 0:
                time.sleep(slp)
    except KeyboardInterrupt:
        print("\n停止...")
        # reBot 平滑回零(Galaxea 靠 disconnect 自身回零)
        if args.go and not args.no_return and rb is not None:
            try:
                print("reBot 回零中...")
                cur = dict(last_rb)
                for _ in range(4000):
                    done = True
                    for m in REBOT_ARM_MOTORS:
                        k = f"{m}.pos"
                        if abs(cur[k]) > 0.5:
                            done = False
                            cur[k] -= math.copysign(min(1.5, abs(cur[k])), cur[k])
                    rb.send_action(cur)
                    if grip_follow:
                        drive_gripper(grip_close_eff)
                    if done:
                        break
                    time.sleep(dt)
            except Exception as e:
                print(f"[warn] reBot 回零失败: {e}")
    finally:
        try:
            if rb is not None:
                rb.disconnect()
        finally:
            try:
                if gx is not None:
                    gx.disconnect()           # Galaxea 自身平滑回零
            finally:
                leader.disconnect()
        print("已安全断开。")


if __name__ == "__main__":
    sys.exit(main())

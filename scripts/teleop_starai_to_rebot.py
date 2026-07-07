#!/usr/bin/env python
"""StarAI Violin (leader) -> Seeed reBot B601-RS (follower, RobStride) 关节空间遥操作桥接.

reBot 官方 follower ``seeed_b601_rs_follower`` 的 send_action 需要 **每个电机名的绝对角度(度)**
(shoulder_pan.pos ... wrist_roll.pos + gripper.pos),而 StarAI 主臂输出 ``joint_1..6.pos``(零位为0的
相对角度,度)+ ``gripper.pos``(行程比例 [0,1])。本脚本做映射并叠加安全措施。

三个模式:
  1) 扫描从臂行程(记录每关节实际 min/max 到 sidecar,供 --match-range 用;夹爪也记开/合):
       python teleop_starai_to_rebot.py --sweep
  2) dry-run(默认,从臂松力矩只打印,验方向):
       python teleop_starai_to_rebot.py
  3) 正式跟随:
       python teleop_starai_to_rebot.py --go --match-range                 # 限速(默认 max-step 3°)
       python teleop_starai_to_rebot.py --go --match-range --no-limit       # 不限速(MIT kp/kd 平滑)

夹爪:官方 RS 插件把夹爪写残了(位置被忽略),本脚本 **直驱 7 号电机** 让它跟随主臂(带力矩上限+限速)。
CAN:本机 PCAN = can5(需先 `sudo ip link set can5 up type can bitrate 1000000 restart-ms 100`)。
⚠️ 安全:臂固定牢、1 米净空、你在场手放急停。
"""
import argparse
import json
import math
import os
import signal
import sys
import time

LEADER_KEYS = [f"joint_{i + 1}.pos" for i in range(6)]                       # StarAI joint_1..6
REBOT_ARM_MOTORS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_yaw", "wrist_roll"]
GRIPPER_MOTOR = "gripper"
ALL_MOTORS = REBOT_ARM_MOTORS + [GRIPPER_MOTOR]
RANGE_FILE_DEFAULT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "rebot_follower_range.json")


def do_sweep(follower, range_file):
    """柔顺扫描:电机保持使能但发零力矩(kp=kd=tau=0)→ 手能拨动且仍回反馈;记录 min/max。

    关键:RobStride 电机被 disable 后不回反馈(get_state 卡在连接时旧值),必须保持使能+零力矩。
    """
    print("\n=== 扫描从臂行程 ===")
    print("电机将保持零力矩(手能拨动、仍上报位置)。按 Enter 开始, 把【每个关节 + 夹爪都缓慢转到两端极限】。")
    print("⚠️ 零力矩下重的关节可能受重力下垂——手扶着扫。")
    input("Enter 开始...")
    lo = {m: math.inf for m in ALL_MOTORS}
    hi = {m: -math.inf for m in ALL_MOTORS}
    print("记录中... 所有关节都扫满后按 Ctrl-C 结束。")
    try:
        while True:
            # 零力矩保持使能 + 请求反馈
            for m in ALL_MOTORS:
                mot = follower.motors.get(m)
                if mot is not None:
                    mot.send_mit(0.0, 0.0, 0.0, 0.0, 0.0)   # 零刚度零力矩 = 限位软 + 上报反馈
                    mot.request_feedback()
            for _ in range(8):
                try:
                    follower.bus.poll_feedback_once()
                except Exception:
                    pass
                time.sleep(0.003)
            for m in ALL_MOTORS:
                mot = follower.motors.get(m)
                st = mot.get_state() if mot is not None else None
                if st is None or getattr(st, "pos", None) is None:
                    continue
                v = math.degrees(st.pos)
                lo[m] = min(lo[m], v)
                hi[m] = max(hi[m], v)
            line = "  ".join(f"{m[:5]}[{lo[m]:+6.1f},{hi[m]:+6.1f}]" for m in ALL_MOTORS)
            print(line, end="\r")
    except KeyboardInterrupt:
        pass
    rng = {m: [lo[m], hi[m]] for m in ALL_MOTORS if lo[m] != math.inf}
    with open(range_file, "w") as f:
        json.dump(rng, f, indent=2)
    print(f"\n已保存从臂行程到 {range_file}")
    for m in ALL_MOTORS:
        if m in rng:
            print(f"  {m:14s} min={rng[m][0]:+7.1f}  max={rng[m][1]:+7.1f}  span={rng[m][1]-rng[m][0]:6.1f}")


def main():
    ap = argparse.ArgumentParser(description="StarAI Violin -> Seeed reBot B601-RS teleop (桥接)")
    ap.add_argument("--leader-port", default="/dev/ttyCH341USB0")
    ap.add_argument("--leader-id", default="leader1")
    ap.add_argument("--can", default="can5", help="reBot PCAN socketcan 接口 (本机为 can5)")
    ap.add_argument("--follower-id", default="follower1")
    ap.add_argument("--freq", type=float, default=30.0)
    ap.add_argument("--flip", default="", help="需翻转方向的臂关节, 如 '3,4' (1-6)")
    ap.add_argument("--scale", type=float, default=1.0, help="主->从 角度增益 (1.0=1:1)")
    ap.add_argument("--match-range", action="store_true",
                    help="按 span 比例匹配行程(主臂满扫范围<->从臂行程),零位对齐。优先用 --sweep 存的实测行程")
    ap.add_argument("--range-file", default=RANGE_FILE_DEFAULT, help="从臂实测行程 sidecar")
    ap.add_argument("--max-step-deg", type=float, default=3.0,
                    help="从臂每步相对步长上限(度)=安全阀。配合 --no-limit 可解除")
    ap.add_argument("--no-limit", action="store_true", help="解除相对步长限制(不限速;靠 MIT kp/kd 平滑)")
    # 夹爪(直驱 7 号电机)
    ap.add_argument("--no-grip", action="store_true", help="不驱动夹爪(默认 --go 时夹爪跟随)")
    ap.add_argument("--grip-close-deg", type=float, default=None, help="夹爪比例=0 对应角度(度);默认取 sweep 的 min")
    ap.add_argument("--grip-open-deg", type=float, default=None, help="夹爪比例=1 对应角度(度);默认取 sweep 的 max")
    ap.add_argument("--grip-kp", type=float, default=9.0, help="夹爪 MIT kp(越大越硬/越灵敏,过大易夹坏/过热)")
    ap.add_argument("--grip-kd", type=float, default=0.3)
    ap.add_argument("--grip-max-step-deg", type=float, default=25.0, help="夹爪每步设定点变化上限(度), 越大越跟手")
    ap.add_argument("--grip-clamp-deg", type=float, default=25.0,
                    help="闭合端过冲(度): ratio=0 时目标压过闭合位这么多, 产生夹持力(越大夹越紧, 过大易过热)")
    ap.add_argument("--grip-ratio-min", type=float, default=0.0,
                    help="主臂夹爪'捏到底'对应的 ratio(实测: 捏最紧时的 leader_ratio, 如 0.62)。映射会把 [min,max] 拉满到从臂全行程")
    ap.add_argument("--grip-ratio-max", type=float, default=1.0, help="主臂夹爪'完全张开'对应的 ratio(实测, 一般 1.0)")
    # 退出回零
    ap.add_argument("--no-return", action="store_true", help="退出时不自动回零(默认 --go 时 Ctrl-C 平滑回零)")
    ap.add_argument("--return-speed-deg-s", type=float, default=35.0, help="退出回零速度(度/秒)")
    # 模式
    ap.add_argument("--sweep", action="store_true", help="扫描从臂行程并保存, 然后退出")
    ap.add_argument("--grip-debug", action="store_true", help="实时打印 leader夹爪ratio + 从臂夹爪当前位置 + 目标")
    ap.add_argument("--go", action="store_true", help="真正驱动从臂(默认 dry-run: 松力矩只打印)")
    args = ap.parse_args()

    sign = [1, 1, 1, 1, 1, 1]
    for tok in args.flip.split(","):
        if tok.strip():
            sign[int(tok) - 1] = -1

    from lerobot.teleoperators.starai_violin_leader import StaraiViolinLeader, StaraiViolinLeaderConfig
    from lerobot.utils.import_utils import register_third_party_plugins
    register_third_party_plugins()
    from lerobot_robot_seeed_b601.seeed_b601_rs_follower import SeeedB601RSFollower
    from lerobot_robot_seeed_b601.config_seeed_b601_rs_follower import SeeedB601RSFollowerConfig

    max_rel = None if args.no_limit else args.max_step_deg
    follower_cfg = SeeedB601RSFollowerConfig(
        port=args.can, can_adapter="socketcan", id=args.follower_id, max_relative_target=max_rel,
    )
    follower = SeeedB601RSFollower(follower_cfg)

    # ---- 扫描模式:只连从臂 ----
    if args.sweep:
        signal.signal(signal.SIGINT, signal.default_int_handler)
        follower.connect()
        try:
            do_sweep(follower, args.range_file)
        finally:
            follower.disconnect()
        return 0

    # ---- 载入从臂实测行程(优先)/ 否则用 config joint_limits ----
    swept = {}
    if os.path.exists(args.range_file):
        with open(args.range_file) as f:
            swept = json.load(f)

    def follower_span(m):
        # 臂关节增益用 config 安全限位(send_action 就是夹到这里, 用扫描大范围会产生死区/越界)。
        lo, hi = follower_cfg.joint_limits[m]
        return abs(hi - lo)

    # ---- match-range 增益 ----
    gain = [args.scale] * 6
    if args.match_range:
        lpath = os.path.expanduser(
            f"~/.cache/huggingface/lerobot/calibration/teleoperators/starai_violin_leader/{args.leader_id}.json")
        with open(lpath) as f:
            lc = json.load(f)
        lmin, lmax = lc["range_min_deg"], lc["range_max_deg"]
        for i, m in enumerate(REBOT_ARM_MOTORS):
            lspan = abs(lmax[i] - lmin[i])
            gain[i] = args.scale * (follower_span(m) / lspan if lspan > 1e-6 else 1.0)

    # ---- 夹爪开合角度(默认取 sweep 的 min/max)----
    grip_close = args.grip_close_deg
    grip_open = args.grip_open_deg
    if GRIPPER_MOTOR in swept and abs(swept[GRIPPER_MOTOR][1] - swept[GRIPPER_MOTOR][0]) >= 5.0:
        if grip_close is None:
            grip_close = swept[GRIPPER_MOTOR][0]
        if grip_open is None:
            grip_open = swept[GRIPPER_MOTOR][1]
    if grip_close is None or grip_open is None:
        # 没有有效 sweep 夹爪范围: 用 config 限位, 并提示需实测
        glo, ghi = follower_cfg.joint_limits[GRIPPER_MOTOR]
        grip_close = glo if grip_close is None else grip_close
        grip_open = ghi if grip_open is None else grip_open
    grip_follow = args.go and not args.no_grip
    # 闭合端过冲: 把 ratio=0 的目标压过闭合位, 让电机持续保持夹持力
    close_dir = -1.0 if grip_close <= grip_open else 1.0
    grip_close_eff = grip_close + close_dir * args.grip_clamp_deg

    mode = "正式跟随 (LIVE)" if args.go else "DRY-RUN (从臂松力矩, 只打印)"
    print("=" * 78)
    print(f"  StarAI Violin -> Seeed reBot B601-RS   [{mode}]")
    print(f"  can={args.can}  sign={sign}  freq={args.freq}Hz  max_step={'无限速' if args.no_limit else str(args.max_step_deg)+'deg'}")
    print(f"  match_range={args.match_range}  swept_range={'有' if swept else '无(用config限位)'}")
    if args.match_range:
        print("  增益: " + "  ".join(f"{REBOT_ARM_MOTORS[i]}={gain[i]:.2f}" for i in range(6)))
    print(f"  夹爪跟随={grip_follow}  (ratio0->{grip_close_eff:.0f}deg[含{args.grip_clamp_deg:.0f}°夹持过冲], ratio1->{grip_open:.0f}deg, kp={args.grip_kp})")
    print("=" * 78)

    def leader_to_arm(la):
        # 只含 6 个臂关节;夹爪单独直驱 7 号电机
        return {f"{m}.pos": sign[i] * gain[i] * float(la[LEADER_KEYS[i]]) for i, m in enumerate(REBOT_ARM_MOTORS)}

    leader = StaraiViolinLeader(StaraiViolinLeaderConfig(port=args.leader_port, id=args.leader_id))

    signal.signal(signal.SIGINT, signal.default_int_handler)
    leader.connect()
    follower.connect()

    if not args.go:
        try:
            follower.disable_torque()
            print("从臂已松力矩(dry-run)。慢扳主臂各关节核对方向,反的记进 --flip。Ctrl-C 停。\n")
        except Exception as e:
            print(f"[warn] disable_torque 失败: {e}")

    def drive_gripper(target_deg):
        """限速驱动夹爪到 target_deg(度), 返回新设定点。"""
        nonlocal grip_set
        if grip_set is None:
            grip_set = target_deg
        d = max(-args.grip_max_step_deg, min(args.grip_max_step_deg, target_deg - grip_set))
        grip_set += d
        gm = follower.motors.get(GRIPPER_MOTOR)
        if gm is not None:
            gm.send_mit(math.radians(grip_set), 0.0, args.grip_kp, args.grip_kd, 0.0)

    def return_to_zero():
        """退出时: 从当前指令平滑地把 6 臂关节回到零位(夹爪回到 close 端)。"""
        ret_step = max(0.2, args.return_speed_deg_s / args.freq)     # 每周期度数
        cur = {f"{m}.pos": last_arm.get(f"{m}.pos", 0.0) for m in REBOT_ARM_MOTORS}
        print("回零中...(手别挡着)")
        for _ in range(4000):
            done = True
            for m in REBOT_ARM_MOTORS:
                k = f"{m}.pos"
                v = cur[k]
                if abs(v) > 0.5:
                    done = False
                    cur[k] = v - math.copysign(min(ret_step, abs(v)), v)
            follower.send_action(cur)
            if grip_follow:
                drive_gripper(grip_close_eff)                        # 夹爪回到闭合端
            if done:
                break
            time.sleep(dt)

    grip_set = None                 # 夹爪设定点(度)
    last_arm = {f"{m}.pos": 0.0 for m in REBOT_ARM_MOTORS}
    t0_dbg = 0.0
    dt = 1.0 / args.freq
    try:
        while True:
            t0 = time.perf_counter()
            la = leader.get_action()
            arm = leader_to_arm(la)
            if args.go:
                last_arm = arm
                follower.send_action(arm)                           # 只发 6 臂关节(夹爪不进 send_action)
                if grip_follow:
                    raw_ratio = float(la.get("gripper.pos", 0.0))
                    # 把主臂实际能用的 ratio 区间 [min,max] 重映射到 [0,1]
                    denom = max(args.grip_ratio_max - args.grip_ratio_min, 1e-3)
                    ratio = min(1.0, max(0.0, (raw_ratio - args.grip_ratio_min) / denom))
                    tgt = grip_close_eff + ratio * (grip_open - grip_close_eff)
                    drive_gripper(tgt)
                    if args.grip_debug and (time.perf_counter() - t0_dbg) > 0.2:
                        t0_dbg = time.perf_counter()
                        gm = follower.motors.get(GRIPPER_MOTOR)
                        gm.request_feedback() if gm else None
                        for _ in range(4):
                            try:
                                follower.bus.poll_feedback_once()
                            except Exception:
                                pass
                        st = gm.get_state() if gm else None
                        curp = math.degrees(st.pos) if (st and getattr(st, "pos", None) is not None) else float("nan")
                        print(f"grip: leader_ratio={raw_ratio:.3f}->{ratio:.3f}  target={tgt:+7.1f}  setpoint={grip_set:+7.1f}  从臂当前={curp:+7.1f}")
            else:
                obs = follower.get_observation()
                cur = " ".join(f"{obs.get(f'{m}.pos', 0.0):+6.1f}" for m in REBOT_ARM_MOTORS)
                tgt = " ".join(f"{arm[f'{m}.pos']:+6.1f}" for m in REBOT_ARM_MOTORS)
                gr = min(1.0, max(0.0, float(la.get("gripper.pos", 0.0))))
                print(f"  臂目标: {tgt}  grip_ratio={gr:.2f} | 当前: {cur}", end="\r")
            sleep = dt - (time.perf_counter() - t0)
            if sleep > 0:
                time.sleep(sleep)
    except KeyboardInterrupt:
        print("\n停止...")
        if args.go and not args.no_return:
            try:
                return_to_zero()
            except Exception as e:
                print(f"[warn] 回零失败: {e}")
    finally:
        try:
            follower.disconnect()
        finally:
            leader.disconnect()
        print("已安全断开。")


if __name__ == "__main__":
    sys.exit(main())

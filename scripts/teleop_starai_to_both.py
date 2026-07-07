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
# 可移植:优先脚本同目录的 rebot_follower_range.json;可用环境变量 REBOT_RANGE_FILE 覆盖
RANGE_FILE = os.environ.get(
    "REBOT_RANGE_FILE", os.path.join(os.path.dirname(os.path.abspath(__file__)), "rebot_follower_range.json")
)
HF_CALIB = os.path.join(
    os.environ.get("HF_LEROBOT_HOME", os.path.expanduser("~/.cache/huggingface/lerobot")), "calibration"
)


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
    ap.add_argument("--galaxea-grip-margin", type=float, default=0.0,
                    help="Galaxea 夹爪两端留白(缩多少行程), 0=夹到底; 默认0.10 夹不紧, 这里默认放到0")
    ap.add_argument("--galaxea-grip-kp", type=float, default=15.0, help="Galaxea 夹爪 kp(越大夹越紧, 过大易过热错误12)")
    # reBot
    ap.add_argument("--rebot-can", default="can5")
    ap.add_argument("--rebot-id", default="follower1")
    ap.add_argument("--rebot-flip", default="", help="reBot 反向关节(单验已确认全对, 一般空)")
    ap.add_argument("--match-range", action="store_true", help="reBot 行程按 config 限位匹配, 零位对齐")
    ap.add_argument("--rebot-scale", type=float, default=1.0)
    ap.add_argument("--max-step-deg", type=float, default=3.0, help="reBot 每步限速(度)")
    ap.add_argument("--no-limit", action="store_true", help="reBot 不限速")
    ap.add_argument("--speed-deg-s", type=float, default=None,
                    help="统一两臂速度上限(度/秒): Galaxea 速度限=此值, reBot 步长=此值/freq。一个旋钮调两臂,天然一致")
    # reBot 夹爪(直驱 7 号)
    ap.add_argument("--grip-kp", type=float, default=9.0)
    ap.add_argument("--grip-kd", type=float, default=0.3)
    ap.add_argument("--grip-max-step-deg", type=float, default=25.0)
    ap.add_argument("--grip-clamp-deg", type=float, default=25.0)
    ap.add_argument("--grip-ratio-min", type=float, default=0.62)
    ap.add_argument("--grip-ratio-max", type=float, default=1.0)
    ap.add_argument("--no-grip", action="store_true")
    ap.add_argument("--diag", action="store_true", help="每2s记录 reBot 各电机故障码+指令是否成功(诊断 30s 断电)")
    # 相机 + rerun(每臂一个独立 rerun 窗口)
    ap.add_argument("--display-data", action="store_true", help="每臂弹一个 rerun 窗口显示相机+关节")
    ap.add_argument("--galaxea-cam", default=None, help="Galaxea 腕部 Orbbec 序列号")
    ap.add_argument("--rebot-cam", default=None, help="reBot 腕部 Orbbec 序列号")
    ap.add_argument("--cam-w", type=int, default=640)
    ap.add_argument("--cam-h", type=int, default=480)
    ap.add_argument("--cam-depth", action="store_true", help="相机也出深度(USB2 相机可能需 mjpg/降帧)")
    ap.add_argument("--cam-format", default="mjpg", choices=["mjpg", "auto", "rgb", "yuyv"],
                    help="彩色流格式; USB2 链路必须 mjpg(压缩), auto/rgb 未压缩会撑爆 USB2 带宽导致取帧超时崩溃")
    args = ap.parse_args()

    def _cam_cfg(sn):
        from lerobot.cameras.orbbec import OrbbecCameraConfig
        return {"wrist": OrbbecCameraConfig(serial_number_or_name=sn, fps=30, width=args.cam_w,
                                            height=args.cam_h, use_depth=args.cam_depth,
                                            color_format=("mjpg" if args.cam_depth else args.cam_format))}

    # 统一速度旋钮:一个值同时定两臂限速(天然一致)
    if args.speed_deg_s:
        args.galaxea_max_vel_deg_s = args.speed_deg_s
        args.max_step_deg = args.speed_deg_s / args.freq
        args.no_limit = False   # 用统一限速, 关掉 reBot 无限速

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
            gripper_margin=args.galaxea_grip_margin, gripper_kp=args.galaxea_grip_kp,
            cameras=(_cam_cfg(args.galaxea_cam) if args.display_data and args.galaxea_cam else {}),
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
            cameras=(_cam_cfg(args.rebot_cam) if args.display_data and args.rebot_cam else {}),
        )
        rb = SeeedB601RSFollower(rb_cfg)
        if args.match_range:
            lp = os.path.join(HF_CALIB, "teleoperators", "starai_violin_leader", f"{args.leader_id}.json")
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

    # ---- 锚点映射: reBot 以"启动时自身姿态"为 home, leader 增量叠加(与 Galaxea 的 home+leader_deg 同款语义)----
    # 修复: 旧版 reBot=gain*leader 无 home 偏移(单边关节遇负角夹到0出现死区)且 gain≠1(与 Galaxea 幅度不一致)。
    rb_home = [0.0] * 6
    if use_rb:
        try:
            _obs = rb.get_observation()
            rb_home = [float(_obs[f"{m}.pos"]) for m in REBOT_ARM_MOTORS]
        except Exception as e:
            print(f"[warn] 读 reBot 当前姿态失败, home 退回 0: {e}")
    _la0 = leader.get_action()
    leader_home = [float(_la0[k]) for k in LEADER_KEYS]
    if use_rb:
        print("  reBot home(deg):   " + " ".join(f"{rb_home[i]:+6.1f}" for i in range(6)))
        print("  leader home(deg):  " + " ".join(f"{leader_home[i]:+6.1f}" for i in range(6)))

    # reBot 映射: 目标 = home + sign*gain*(leader - leader_home)。默认 gain=1 → 与 Galaxea 一样 1:1 增量。
    def rebot_arm(la):
        return {f"{m}.pos": rb_home[i] + rb_sign[i] * rb_gain[i] * (float(la[LEADER_KEYS[i]]) - leader_home[i])
                for i, m in enumerate(REBOT_ARM_MOTORS)}

    grip_set = [None]

    def drive_gripper(target_deg):
        if grip_set[0] is None:
            grip_set[0] = target_deg
        d = max(-args.grip_max_step_deg, min(args.grip_max_step_deg, target_deg - grip_set[0]))
        grip_set[0] += d
        gm = rb.motors.get(GRIPPER_MOTOR)
        if gm is not None:
            gm.send_mit(math.radians(grip_set[0]), 0.0, args.grip_kp, args.grip_kd, 0.0)

    # ---------- rerun:一个窗口, 两条臂并排(galaxea/* 与 rebot/*) ----------
    _rr_on = args.display_data and ((gx is not None and args.galaxea_cam) or (rb is not None and args.rebot_cam))
    if _rr_on:
        import rerun as rr
        rr.init("starai_dual_teleop", spawn=True)   # 单 viewer

    def log_rr(prefix, follower, joints):
        """只读相机(快, 后台线程)+ 指令值画关节曲线(不走 CAN, 不拖慢控制环)。"""
        import rerun as rr
        for cname, cam in getattr(follower, "cameras", {}).items():
            try:
                rr.log(f"{prefix}/image/{cname}", rr.Image(cam.read_latest(max_age_ms=500)))
                if getattr(cam, "use_depth", False):
                    d = cam.read_latest_depth(max_age_ms=500)
                    rr.log(f"{prefix}/depth/{cname}", rr.DepthImage(d[..., 0], meter=1000.0))
            except Exception:
                pass
        for k, v in joints.items():
            rr.log(f"{prefix}/state/{k}", rr.Scalars(float(v)))

    last_rb = {f"{m}.pos": rb_home[i] for i, m in enumerate(REBOT_ARM_MOTORS)}
    _t_start = time.time()
    _t_diag = [0.0]
    _t_rr = [0.0]
    dt = 1.0 / args.freq
    try:
        while True:
            t0 = time.perf_counter()
            la = leader.get_action()
            if args.go:
                if gx is not None:
                    la_gx = dict(la)                         # 夹爪 ratio 同样重映射(主臂捏到底~0.6→满行程)
                    _rr = float(la.get("gripper.pos", 0.0))
                    _dn = max(args.grip_ratio_max - args.grip_ratio_min, 1e-3)
                    la_gx["gripper.pos"] = min(1.0, max(0.0, (_rr - args.grip_ratio_min) / _dn))
                    gx.send_action(la_gx)                    # Galaxea 内部做 leader_deg 映射
                if rb is not None:
                    arm = rebot_arm(la)
                    last_rb = arm
                    try:
                        rb.send_action(arm)
                        rb_send_ok = True
                    except Exception as e:
                        rb_send_ok = False
                        print(f"\n[diag] t={time.time()-_t_start:.1f}s rb.send_action 抛异常: {type(e).__name__}: {str(e)[:80]}", flush=True)
                    if grip_follow:
                        rr = float(la.get("gripper.pos", 0.0))
                        denom = max(args.grip_ratio_max - args.grip_ratio_min, 1e-3)
                        ratio = min(1.0, max(0.0, (rr - args.grip_ratio_min) / denom))
                        drive_gripper(grip_close_eff + ratio * (grip_open - grip_close_eff))
                    if args.diag and time.time() - _t_diag[0] >= 2.0:
                        _t_diag[0] = time.time()
                        drop, fault = [], []
                        for _n, _m in rb.motors.items():
                            try:
                                _fr = _m.robstride_get_fault_report()
                                if _fr != (0, 0):
                                    fault.append(f"{_n}={_fr}")
                            except Exception:
                                drop.append(_n)
                        tag = (f" 掉线(疑USB/power):{drop}" if drop else "") + (f" 故障:{fault}" if fault else "")
                        print(f"\n[diag] t={time.time()-_t_start:.1f}s send_ok={rb_send_ok}{tag or ' 电机正常'}", flush=True)
            else:
                parts = []
                if gx is not None:
                    tg = gx._resolve_target(la)
                    parts.append("GX目标(deg): " + " ".join(f"{np.degrees(t):+6.1f}" for t in tg))
                if rb is not None:
                    arm = rebot_arm(la)
                    parts.append("RB目标(deg): " + " ".join(f"{arm[f'{m}.pos']:+6.1f}" for m in REBOT_ARM_MOTORS))
                print("  " + "  |  ".join(parts), end="\r")
            # rerun:节流 ~10Hz, 只读相机(快)+ 指令值画关节, 不拖慢控制环
            if _rr_on and args.go and time.time() - _t_rr[0] >= 0.1:
                _t_rr[0] = time.time()
                try:
                    if gx is not None and args.galaxea_cam:
                        log_rr("galaxea", gx, {k: la[k] for k in LEADER_KEYS})
                    if rb is not None and args.rebot_cam:
                        log_rr("rebot", rb, {m: last_rb[f"{m}.pos"] for m in REBOT_ARM_MOTORS})
                except Exception as e:
                    print(f"\n[rerun] log 失败: {str(e)[:60]}", flush=True)
            slp = dt - (time.perf_counter() - t0)
            if slp > 0:
                time.sleep(slp)
    except KeyboardInterrupt:
        print("\n停止...")
        # reBot 平滑回零(Galaxea 靠 disconnect 自身回零)
        if args.go and not args.no_return and rb is not None:
            try:
                print("reBot 回 home 姿态中...")
                cur = dict(last_rb)
                for _ in range(4000):
                    done = True
                    for i, m in enumerate(REBOT_ARM_MOTORS):
                        k = f"{m}.pos"
                        err = rb_home[i] - cur[k]
                        if abs(err) > 0.5:
                            done = False
                            cur[k] += math.copysign(min(1.5, abs(err)), err)
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

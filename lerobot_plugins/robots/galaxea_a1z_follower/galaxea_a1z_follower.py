#!/usr/bin/env python
# Copyright 2026 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

import json
import logging
import threading
import time
from pathlib import Path

import numpy as np

from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected

from ..robot import Robot
from .config_galaxea_a1z_follower import GalaxeaA1ZFollowerConfig

logger = logging.getLogger(__name__)

NUM_JOINTS = 6


class GalaxeaA1ZFollower(Robot):
    """Galaxea A1Z 6-DoF follower (+gripper) over CAN, wrapping the ``a1z`` SDK.

    Arm keys ``joint_1.pos`` ... ``joint_6.pos`` (rad) are driven by the a1z ArmRobot;
    the gripper (7th MotorB at ``gripper_can_id``, which the A1Z SDK omits) is driven
    on a SEPARATE SocketCAN socket. Calibration records the arm zero+range AND the
    gripper open/close positions. Teleop maps the leader's normalized [-1,1] inputs
    into each side's calibrated range; targets are rate-limited and clamped.
    """

    config_class = GalaxeaA1ZFollowerConfig
    name = "galaxea_a1z_follower"

    def __init__(self, config: GalaxeaA1ZFollowerConfig):
        # set before super().__init__ (may auto-load calibration)
        self._home = None
        self._range_min = None
        self._range_max = None
        self._grip_min = None   # gripper closed/open extremes (rad)
        self._grip_max = None
        super().__init__(config)
        self.config = config
        self._robot = None       # a1z ArmRobot (6 joints)
        self._grip = None        # MotorB gripper
        self._grip_bus = None
        self._keys = [f"joint_{i + 1}.pos" for i in range(NUM_JOINTS)]
        self._sign = np.asarray(config.joint_sign, dtype=float)
        self._cmd_pos = None
        self._grip_cmd = None    # gripper commanded setpoint (rad)
        self._jmin = None
        self._jmax = None
        self._last_t = None      # last send_action time, for velocity limiting

    @property
    def _has_grip(self) -> bool:
        return self.config.enable_gripper

    @property
    def _joints_ft(self) -> dict[str, type]:
        ft = {k: float for k in self._keys}
        if self._has_grip:
            ft["gripper.pos"] = float
        return ft

    @property
    def observation_features(self) -> dict[str, type]:
        return self._joints_ft

    @property
    def action_features(self) -> dict[str, type]:
        return self._joints_ft

    @property
    def is_connected(self) -> bool:
        return self._robot is not None and self._robot.is_running

    # ---- gripper helpers ----
    def _open_gripper(self) -> None:
        import can
        from a1z.motor_drivers.motor_b_driver import MotorB, MotorBRanges

        self._grip_bus = can.Bus(channel=self.config.can_channel, interface="socketcan", bitrate=1_000_000)
        self._grip = MotorB(motor_id=self.config.gripper_can_id, bus=self._grip_bus, ranges=MotorBRanges())
        self._grip.enable()
        time.sleep(0.05)
        pos = self._read_grip_pos()
        self._grip_cmd = pos if pos is not None else 0.0

    def _read_grip_pos(self, timeout: float = 0.15):
        """Read the latest gripper feedback position (rad) from our socket, or None."""
        pos = None
        t0 = time.perf_counter()
        while time.perf_counter() - t0 < timeout:
            msg = self._grip_bus.recv(timeout=timeout)
            if msg is None:
                break
            if msg.arbitration_id == self.config.gripper_can_id:
                fb = self._grip.parse_feedback(msg)
                if fb is not None:
                    pos = fb.position
        return pos

    def _grip_compliant(self) -> None:
        """Command zero-stiffness so the gripper can be hand-moved during calibration."""
        cur = self._read_grip_pos() or 0.0
        self._grip.send_mit_command(cur, 0.0, 0.0, 0.1, 0.0)

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        from a1z.robots.get_robot import get_a1z_robot

        self._robot = get_a1z_robot(
            can_channel=self.config.can_channel,
            gravity_comp_factor=self.config.gravity_comp_factor,
            zero_gravity_mode=False,
            control_freq_hz=self.config.control_freq_hz,
        )
        self._robot.start()
        self._cmd_pos = np.asarray(self._robot.get_joint_pos(), dtype=float)

        limits = self._robot.get_robot_info().get("joint_limits")
        if not limits:
            raise RuntimeError(f"{self}: a1z SDK returned no joint_limits; cannot map teleop safely.")
        self._jmin = np.array([lo for lo, hi in limits], dtype=float)
        self._jmax = np.array([hi for lo, hi in limits], dtype=float)

        if self._has_grip:
            self._open_gripper()

        if not self.is_calibrated and calibrate:
            logger.info(f"{self} not calibrated; running calibration.")
            self.calibrate()
        logger.info(f"{self} connected on {self.config.can_channel}. "
                    f"calibrated={self.is_calibrated} gripper={self._has_grip}")

    @property
    def is_calibrated(self) -> bool:
        arm_ok = self._home is not None and self._range_min is not None and self._range_max is not None
        grip_ok = (not self.config.enable_gripper) or (self._grip_min is not None and self._grip_max is not None)
        return arm_ok and grip_ok

    def calibrate(self) -> None:
        # --- arm: zero + range (compliant, limit e-stop disabled during sweep) ---
        cur = np.asarray(self._robot.get_joint_pos(), dtype=float)
        self._robot.command_joint_state({"pos": cur, "vel": np.zeros(NUM_JOINTS), "kp": np.zeros(NUM_JOINTS)})
        input(f"\n[{self}] 从臂已柔顺悬浮。① 零位:扶住移到【零位姿态】(与主臂对应),按 Enter...")
        self._home = np.asarray(self._robot.get_joint_pos(), dtype=float)
        print(f"零位记录(deg): {np.round(np.degrees(self._home),1).tolist()}")

        input("② 臂限位:按 Enter 开始,把每个关节都缓慢转到两端极限来回扫一遍...")
        print("记录中... 所有关节都转满后按 Enter 结束。")
        self._robot._joint_limits = None
        mins = self._home.copy()
        maxs = self._home.copy()
        done = threading.Event()
        threading.Thread(target=lambda: (input(), done.set()), daemon=True).start()
        while not done.is_set():
            pos = np.asarray(self._robot.get_joint_pos(), dtype=float)
            mins = np.minimum(mins, pos)
            maxs = np.maximum(maxs, pos)
            print("  min(deg): " + " ".join(f"{np.degrees(m):+6.1f}" for m in mins)
                  + " | max: " + " ".join(f"{np.degrees(m):+6.1f}" for m in maxs), end="\r")
            time.sleep(0.02)
        self._range_min = np.maximum(mins, self._jmin)
        self._range_max = np.minimum(maxs, self._jmax)

        # --- gripper: open/close range (compliant hand-move) ---
        if self._has_grip:
            input("\n③ 夹爪:按 Enter 开始,把夹爪【完全张开】和【完全闭合】各做一次...")
            print("记录中... 夹爪开合到两端后按 Enter 结束。")
            gmin = gmax = self._read_grip_pos() or 0.0
            gdone = threading.Event()
            threading.Thread(target=lambda: (input(), gdone.set()), daemon=True).start()
            while not gdone.is_set():
                self._grip_compliant()
                p = self._read_grip_pos()
                if p is not None:
                    gmin = min(gmin, p); gmax = max(gmax, p)
                    print(f"  夹爪范围(rad): [{gmin:+.2f}, {gmax:+.2f}]", end="\r")
                time.sleep(0.02)
            self._grip_min, self._grip_max = gmin, gmax
            self._grip_cmd = self._read_grip_pos() or gmin

        self._save_calibration()
        print(f"\n标定完成,已保存到 {self.calibration_fpath}")
        logger.info(f"{self} home(deg)={np.round(np.degrees(self._home),1).tolist()} "
                    f"arm_range=[{np.round(np.degrees(self._range_min),0).tolist()},"
                    f"{np.round(np.degrees(self._range_max),0).tolist()}] "
                    f"grip=[{self._grip_min},{self._grip_max}]")

    def configure(self) -> None:
        pass

    def _load_calibration(self, fpath: Path | None = None) -> None:
        fpath = self.calibration_fpath if fpath is None else fpath
        with open(fpath) as f:
            d = json.load(f)
        self._home = np.asarray(d["home_rad"], dtype=float)
        self._range_min = np.asarray(d["range_min_rad"], dtype=float)
        self._range_max = np.asarray(d["range_max_rad"], dtype=float)
        if "grip_min_rad" in d:
            self._grip_min = float(d["grip_min_rad"])
            self._grip_max = float(d["grip_max_rad"])

    def _save_calibration(self, fpath: Path | None = None) -> None:
        fpath = self.calibration_fpath if fpath is None else fpath
        d = {"home_rad": self._home.tolist(),
             "range_min_rad": self._range_min.tolist(),
             "range_max_rad": self._range_max.tolist()}
        if self._grip_min is not None:
            d["grip_min_rad"] = float(self._grip_min)
            d["grip_max_rad"] = float(self._grip_max)
        with open(fpath, "w") as f:
            json.dump(d, f, indent=4)

    @check_if_not_connected
    def get_observation(self) -> dict[str, float]:
        pos = np.asarray(self._robot.get_joint_pos(), dtype=float)
        obs = {k: float(pos[i]) for i, k in enumerate(self._keys)}
        if self._has_grip:
            gp = self._read_grip_pos(timeout=0.005)
            obs["gripper.pos"] = float(gp if gp is not None else (self._grip_cmd or 0.0))
        return obs

    def _resolve_target(self, action: dict[str, float]) -> np.ndarray:
        vals = np.array([float(action[k]) for k in self._keys], dtype=float)
        if self.config.teleop_input == "rad":
            return np.clip(vals, self._jmin, self._jmax)
        # "leader_deg": 1:1 direct angle from the leader's degrees-from-zero.
        target = self._home + self._sign * self.config.scale * np.deg2rad(vals)
        # Clamp to the calibrated working range (falls back to hardware limits).
        lo = np.maximum(self._range_min, self._jmin)
        hi = np.minimum(self._range_max, self._jmax)
        # ① soft joint limit: cap travel from zero to ±max_joint_travel_deg
        if self.config.max_joint_travel_deg is not None:
            trav = np.deg2rad(self.config.max_joint_travel_deg)
            lo = np.maximum(lo, self._home - trav)
            hi = np.minimum(hi, self._home + trav)
        return np.clip(target, lo, hi)

    def _command_gripper(self, action: dict[str, float]) -> None:
        if not self._has_grip or "gripper.pos" not in action or self._grip_min is None:
            return
        # leader gripper is a travel fraction [0,1]; sign<0 flips which end is open/close
        frac = np.clip(float(action["gripper.pos"]), 0.0, 1.0)
        if self.config.gripper_sign < 0:
            frac = 1.0 - frac
        # keep a margin off both hard stops to avoid stall/overheat
        m = self.config.gripper_margin
        lo = self._grip_min + m * (self._grip_max - self._grip_min)
        hi = self._grip_max - m * (self._grip_max - self._grip_min)
        gtarget = lo + frac * (hi - lo)
        gstep = self.config.gripper_max_step_rad
        step = np.clip(gtarget - self._grip_cmd, -gstep, gstep)
        self._grip_cmd = self._grip_cmd + step
        self._grip.send_mit_command(self._grip_cmd, 0.0, self.config.gripper_kp, self.config.gripper_kd, 0.0)

    @check_if_not_connected
    def send_action(self, action: dict[str, float]) -> dict[str, float]:
        target = self._resolve_target(action)
        # ② velocity limit: cap setpoint change by max_joint_vel * dt (hard backstop = max_step_rad)
        now = time.perf_counter()
        dt = (now - self._last_t) if self._last_t is not None else 0.02
        self._last_t = now
        max_step = min(self.config.max_step_rad, np.deg2rad(self.config.max_joint_vel_deg_s) * dt)
        step = np.clip(target - self._cmd_pos, -max_step, max_step)
        self._cmd_pos = self._cmd_pos + step
        self._robot.command_joint_pos(self._cmd_pos)
        self._command_gripper(action)
        sent = {k: float(self._cmd_pos[i]) for i, k in enumerate(self._keys)}
        if self._has_grip and self._grip_cmd is not None:
            sent["gripper.pos"] = float(self._grip_cmd)
        return sent

    def _return_to_zero(self) -> None:
        """Smoothly CLOSE the gripper and move the arm back to its zero pose."""
        if self._grip is not None and self._grip_min is not None:
            m = self.config.gripper_margin
            span = self._grip_max - self._grip_min
            # closed end (opposite of open): matches leader-closed in _command_gripper
            close_t = (self._grip_max - m * span) if self.config.gripper_sign < 0 else (self._grip_min + m * span)
            cur = self._grip_cmd if self._grip_cmd is not None else (self._read_grip_pos() or close_t)
            for _ in range(25):
                cur = cur + np.clip(close_t - cur, -0.15, 0.15)
                self._grip.send_mit_command(cur, 0.0, min(self.config.gripper_kp, 10.0), self.config.gripper_kd, 0.0)
                time.sleep(0.03)
                if abs(close_t - cur) < 1e-3:
                    break
        # smooth minimum-jerk move to the zero pose
        self._robot.move_joints(self._home, speed=np.deg2rad(self.config.return_speed_deg_s))

    def disconnect(self) -> None:
        # ③ return to zero pose + open gripper before powering down (avoid bad posture)
        if (self.config.return_to_zero_on_exit and self._robot is not None
                and self._robot.is_running and not self._robot.is_estopped and self._home is not None):
            try:
                logger.info(f"{self} returning to zero pose...")
                self._return_to_zero()
            except Exception as e:
                logger.warning(f"{self} return-to-zero failed: {e}")
        if self._grip is not None:
            try:
                self._grip.disable()
            except Exception:  # nosec B110
                pass
        if self._grip_bus is not None:
            try:
                self._grip_bus.shutdown()
            except Exception:  # nosec B110
                pass
            self._grip_bus = None
        self._grip = None
        if self._robot is not None:
            try:
                self._robot.stop()
            finally:
                self._robot = None
        logger.info(f"{self} disconnected.")

    def estop(self) -> None:
        if self._robot is not None:
            self._robot.estop()

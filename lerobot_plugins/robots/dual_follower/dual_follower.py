"""`dual_follower` — one StarAI leader driving Galaxea A1Z + reBot B601-RS at once.

Composite LeRobot ``Robot``: constructs a ``GalaxeaA1ZFollower`` and a
``SeeedB601RSFollower`` internally, and in ``send_action`` fans the leader action
out to both — Galaxea via its own ``leader_deg`` map, reBot via a home-anchored
1:1 delta map (``target = rb_home + sign*scale*(leader - leader_home)``) plus a
direct-drive gripper on motor 7.

Why the home-anchor: Galaxea maps ``home + sign*scale*leader_deg`` (1:1 delta from
its calibrated home). The old reBot map ``gain*leader_deg`` (zero-anchored, no
offset) had the wrong magnitude vs Galaxea AND dead-zones on reBot's one-sided
joints (shoulder_lift[0,170]/elbow_flex[0,200] clamp to 0 on negative leader).
Anchoring reBot on its own startup pose fixes both — matches Galaxea, no dead-zone,
no startup snap (target == current pose at t0, so ``--no-limit`` is safe to start).

Works with the stock CLIs (both followers must be calibrated first under their own
types ``galaxea_a1z_follower`` / ``seeed_b601_rs_follower``):

    lerobot-teleoperate --robot.type=dual_follower ... --teleop.type=starai_violin_leader ...
    lerobot-record      --robot.type=dual_follower ... --teleop.type=starai_violin_leader ... --dataset...
"""

import logging
import math
import time

import numpy as np

from lerobot.robots.robot import Robot
from lerobot.robots.galaxea_a1z_follower import GalaxeaA1ZFollower, GalaxeaA1ZFollowerConfig
from lerobot.cameras.utils import make_cameras_from_configs

from .config_dual_follower import DualFollowerConfig

logger = logging.getLogger(__name__)

LEADER_KEYS = [f"joint_{i + 1}.pos" for i in range(6)]
REBOT_ARM_MOTORS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_yaw", "wrist_roll"]
GRIPPER = "gripper"


def _parse_flip(s: str) -> list[float]:
    sign = [1.0] * 6
    for tok in s.split(","):
        tok = tok.strip()
        if tok:
            sign[int(tok) - 1] = -1.0
    return sign


class DualFollower(Robot):
    config_class = DualFollowerConfig
    name = "dual_follower"

    def __init__(self, config: DualFollowerConfig):
        super().__init__(config)
        self.config = config

        # ---- Galaxea A1Z (lerobot tree plugin) ----
        self.galaxea = GalaxeaA1ZFollower(GalaxeaA1ZFollowerConfig(
            can_channel=config.galaxea_can_channel, id=config.galaxea_id,
            joint_sign=_parse_flip(config.galaxea_flip), scale=config.galaxea_scale,
            max_step_rad=math.radians(config.galaxea_max_step_deg),
            max_joint_vel_deg_s=config.galaxea_max_vel_deg_s,
            return_to_zero_on_exit=config.return_home_on_exit,
            gripper_margin=config.galaxea_gripper_margin, gripper_kp=config.galaxea_gripper_kp,
            cameras={},
        ))

        # ---- reBot B601-RS (external pip plugin lerobot_robot_seeed_b601) ----
        from lerobot_robot_seeed_b601.seeed_b601_rs_follower import SeeedB601RSFollower
        from lerobot_robot_seeed_b601.config_seeed_b601_rs_follower import SeeedB601RSFollowerConfig
        self._rb_cfg = SeeedB601RSFollowerConfig(
            port=config.rebot_can, can_adapter="socketcan", id=config.rebot_id,
            max_relative_target=(None if config.rebot_no_limit else config.rebot_max_step_deg),
            cameras={},
        )
        self.rebot = SeeedB601RSFollower(self._rb_cfg)
        self._rb_sign = _parse_flip(config.rebot_flip)

        # ---- cameras owned by the composite robot (stock loop reads these for rerun/record) ----
        self.cameras = make_cameras_from_configs(config.cameras)

        # runtime state (filled at connect / first send_action)
        self._rb_home = [0.0] * 6
        self._leader_home: list[float] | None = None
        self._last_rb = {f"{m}.pos": 0.0 for m in REBOT_ARM_MOTORS}
        self._grip_close = self._grip_open = self._grip_close_eff = None
        self._grip_set: float | None = None

    # ---------------- features ----------------
    @property
    def _motors_ft(self) -> dict[str, type]:
        ft: dict[str, type] = {}
        for m in REBOT_ARM_MOTORS:
            ft[f"rebot_{m}.pos"] = float
        ft["rebot_gripper.pos"] = float
        for i in range(6):
            ft[f"galaxea_joint_{i + 1}.pos"] = float
        ft["galaxea_gripper.pos"] = float
        return ft

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        return {name: (cam.height, cam.width, 3) for name, cam in self.cameras.items()}

    @property
    def observation_features(self) -> dict:
        return {**self._motors_ft, **self._cameras_ft}

    @property
    def action_features(self) -> dict:
        # action = the leader joint dict consumed by send_action (7 values)
        return {**{k: float for k in LEADER_KEYS}, "gripper.pos": float}

    @property
    def is_connected(self) -> bool:
        cams_ok = all(c.is_connected for c in self.cameras.values())
        return self.galaxea.is_connected and self.rebot.is_connected and cams_ok

    @property
    def is_calibrated(self) -> bool:
        return self.galaxea.is_calibrated and self.rebot.is_calibrated

    # ---------------- lifecycle ----------------
    def connect(self, calibrate: bool = True) -> None:
        self.galaxea.connect(calibrate=calibrate)
        self.rebot.connect()
        for c in self.cameras.values():
            c.connect()

        # reBot gripper close/open range from config joint_limits
        lo, hi = self._rb_cfg.joint_limits[GRIPPER]
        self._grip_close, self._grip_open = lo, hi
        close_dir = -1.0 if self._grip_close <= self._grip_open else 1.0
        self._grip_close_eff = self._grip_close + close_dir * self.config.grip_clamp_deg

        # capture reBot home = its current pose (abs degrees) → anchor for the delta map
        obs = self.rebot.get_observation()
        self._rb_home = [float(obs[f"{m}.pos"]) for m in REBOT_ARM_MOTORS]
        self._last_rb = {f"{m}.pos": self._rb_home[i] for i, m in enumerate(REBOT_ARM_MOTORS)}
        self._leader_home = None
        logger.info(f"{self}: reBot home(deg)={[round(x, 1) for x in self._rb_home]}")

    def calibrate(self) -> None:
        # 组合机器人无自身标定;两条从臂各自标定:
        #   lerobot-calibrate --robot.type=galaxea_a1z_follower --robot.can_channel=can0 --robot.id=follower1
        #   lerobot-calibrate --robot.type=seeed_b601_rs_follower --robot.port=can5 --robot.can_adapter=socketcan --robot.id=follower1
        logger.info(f"{self}: 请分别标定 galaxea_a1z_follower 与 seeed_b601_rs_follower(见 docs)。")

    def configure(self) -> None:
        pass

    # ---------------- IO ----------------
    def get_observation(self) -> dict:
        obs: dict = {}
        g = self.galaxea.get_observation()   # joints in rad
        for i in range(6):
            obs[f"galaxea_joint_{i + 1}.pos"] = float(np.degrees(g.get(f"joint_{i + 1}.pos", 0.0)))
        obs["galaxea_gripper.pos"] = float(g.get("gripper.pos", 0.0))
        r = self.rebot.get_observation()     # joints in deg
        for m in REBOT_ARM_MOTORS:
            obs[f"rebot_{m}.pos"] = float(r.get(f"{m}.pos", 0.0))
        obs["rebot_gripper.pos"] = float(r.get("gripper.pos", 0.0))
        for name, cam in self.cameras.items():
            obs[name] = cam.async_read(timeout_ms=300)
        return obs

    def send_action(self, action: dict) -> dict:
        if self._leader_home is None:
            self._leader_home = [float(action[k]) for k in LEADER_KEYS]

        rr = float(action.get("gripper.pos", 0.0))
        dn = max(self.config.grip_ratio_max - self.config.grip_ratio_min, 1e-3)
        grip_ratio = min(1.0, max(0.0, (rr - self.config.grip_ratio_min) / dn))

        # ---- Galaxea: leader dict passthrough (internal leader_deg map) + gripper ratio remap ----
        la_gx = dict(action)
        la_gx["gripper.pos"] = grip_ratio
        self.galaxea.send_action(la_gx)

        # ---- reBot: home-anchored 1:1 delta ----
        arm = {
            f"{m}.pos": self._rb_home[i]
            + self._rb_sign[i] * self.config.rebot_scale * (float(action[LEADER_KEYS[i]]) - self._leader_home[i])
            for i, m in enumerate(REBOT_ARM_MOTORS)
        }
        self.rebot.send_action(arm)
        self._last_rb = arm

        # ---- reBot gripper: direct-drive motor 7 ----
        if self.config.grip_follow:
            self._drive_gripper(self._grip_close_eff + grip_ratio * (self._grip_open - self._grip_close_eff))

        return action

    def _drive_gripper(self, target_deg: float) -> None:
        if self._grip_set is None:
            self._grip_set = target_deg
        step = max(-self.config.grip_max_step_deg, min(self.config.grip_max_step_deg, target_deg - self._grip_set))
        self._grip_set += step
        gm = self.rebot.motors.get(GRIPPER)
        if gm is not None:
            gm.send_mit(math.radians(self._grip_set), 0.0, self.config.grip_kp, self.config.grip_kd, 0.0)

    def disconnect(self) -> None:
        # reBot 平滑回 home(Galaxea 靠自身 disconnect 回零)
        if self.config.return_home_on_exit and self.rebot.is_connected:
            try:
                cur = dict(self._last_rb)
                for _ in range(4000):
                    done = True
                    for i, m in enumerate(REBOT_ARM_MOTORS):
                        k = f"{m}.pos"
                        err = self._rb_home[i] - cur[k]
                        if abs(err) > 0.5:
                            done = False
                            cur[k] += math.copysign(min(1.5, abs(err)), err)
                    self.rebot.send_action(cur)
                    if self.config.grip_follow:
                        self._drive_gripper(self._grip_close_eff)
                    if done:
                        break
                    time.sleep(1.0 / 30.0)
            except Exception as e:
                logger.warning(f"{self}: reBot 回 home 失败: {e}")
        for c in self.cameras.values():
            try:
                c.disconnect()
            except Exception:
                pass
        try:
            self.rebot.disconnect()
        finally:
            self.galaxea.disconnect()

#!/usr/bin/env python
# Copyright 2026 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

from dataclasses import dataclass, field

from lerobot.cameras import CameraConfig

from ..config import RobotConfig


@RobotConfig.register_subclass("galaxea_a1z_follower")
@dataclass
class GalaxeaA1ZFollowerConfig(RobotConfig):
    """Galaxea A1Z 6-DoF follower arm (CAN, MIT force-control motors).

    Wraps the `a1z` SDK. Runs in position-hold mode (PD + gravity compensation);
    ``send_action`` targets are rate-limited per call for safety.

    Cross-brand teleop mapping is done here so the standard ``lerobot-teleoperate``
    CLI (identity processors) works. With ``teleop_input="norm"`` the incoming action
    is the leader's NORMALIZED joint position in [-100, 100], mapped into THIS arm's
    hardware joint limits (read from the a1z SDK) with per-joint ``joint_sign`` and a
    ``scale`` range-usage fraction. Use ``"rad"`` to command native radians directly
    (e.g. dataset replay). No follower calibration is needed — limits come from the SDK.
    """

    # SocketCAN interface (the HHS USB-CANFD adapter enumerates as can4 here).
    can_channel: str = "can4"
    # Gravity compensation scale (0=off, 1=full).
    gravity_comp_factor: float = 1.0
    # Internal control-loop frequency of the a1z SDK. 150 (not 250) reduces the
    # per-cycle CAN load on the gs_usb USB-CAN adapter, cutting feedback-stall e-stops.
    control_freq_hz: int = 150
    # SAFETY: hard per-call cap on setpoint change (radians). Backstop under the
    # velocity limit below.
    max_step_rad: float = 0.06

    # --- SAFETY FEATURES (all tunable) ---
    # ① Soft joint limit: cap each arm joint's travel from its zero to ±this (deg).
    #    None = calibrated range only (full range, joints can reach their extremes).
    #    Set a number (e.g. 60) to further restrict. Default off.
    max_joint_travel_deg: float | None = None
    # ② Max follower joint speed (deg/s). The setpoint won't move faster than this,
    #    so the arm can't swing fast enough to hurt someone. Lower = safer/slower.
    max_joint_vel_deg_s: float = 50.0
    # ③ On disconnect (teleop end / Ctrl-C), smoothly return the arm to its zero pose
    #    (and open the gripper) so it never stops in a bad posture.
    return_to_zero_on_exit: bool = True
    return_speed_deg_s: float = 25.0

    # --- teleop mapping ---
    # "leader_deg": arm keys are the leader's degrees-from-zero → 1:1 direct angle:
    #   target = home + sign*scale*deg2rad(leader_deg), clamped to the CALIBRATED range
    #   (zero for alignment, range for the clamp). Responsive, not range-compressed.
    # "rad": treat incoming action as native radians (no mapping).
    teleop_input: str = "leader_deg"
    # Cameras (e.g. wrist Orbbec Gemini 305). Captured into observations and shown in
    # rerun with ``--display_data=true``. Example:
    #   --robot.cameras='{ wrist: {type: orbbec, serial_number_or_name: "CV2856D0006R",
    #                              use_depth: true} }'
    cameras: dict[str, CameraConfig] = field(default_factory=dict)

    # Per-joint direction (+1 / -1).
    joint_sign: list[int] = field(default_factory=lambda: [1, 1, 1, 1, 1, 1])
    # Leader->follower angle gain (1.0 = 1:1 degrees).
    scale: float = 1.0

    # --- gripper (7th MotorB, separate socket; the A1Z SDK omits it) ---
    enable_gripper: bool = True
    gripper_can_id: int = 7
    gripper_sign: int = -1         # flip if open/close is reversed vs leader
    # Gripper responsiveness. kp = tracking stiffness/snappiness (too high -> the coil
    # overheats when holding against a hard stop, error 12); 15 is snappy but safe with
    # a margin. gripper_max_step_rad caps travel speed per call (higher = faster).
    gripper_kp: float = 15.0
    gripper_kd: float = 0.4
    gripper_max_step_rad: float = 0.4
    # Shrink the commanded travel by this fraction at EACH end so the gripper doesn't
    # slam/stall into its hard stops (which stalls -> overheats). 0 = full travel.
    gripper_margin: float = 0.10

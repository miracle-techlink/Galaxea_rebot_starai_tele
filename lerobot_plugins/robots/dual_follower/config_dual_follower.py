"""Config for the `dual_follower` composite robot.

One StarAI Violin leader → simultaneously mirrors two follower arms:
  - Galaxea A1Z   (can0, gs_usb, internal ``leader_deg`` map)
  - Seeed reBot B601-RS (can5, PCAN/socketcan, home-anchored 1:1 delta map)

Registered as robot type ``dual_follower`` so it drives with the stock
``lerobot-teleoperate`` / ``lerobot-record`` CLIs, e.g.

    lerobot-teleoperate \
        --robot.type=dual_follower --robot.id=dual1 \
        --robot.galaxea_flip="1,5,6" \
        --robot.cameras='{ galaxea_wrist: {type: orbbec, serial_number_or_name: "CV2856D0006R", fps: 30, width: 640, height: 480, color_format: "mjpg"}, rebot_wrist: {type: orbbec, serial_number_or_name: "CV275610002L", fps: 30, width: 640, height: 480, color_format: "mjpg"} }' \
        --teleop.type=starai_violin_leader --teleop.port=/dev/ttyCH341USB0 --teleop.id=leader1 \
        --display_data=true
"""

from dataclasses import dataclass, field

from lerobot.cameras import CameraConfig

from ..config import RobotConfig


@RobotConfig.register_subclass("dual_follower")
@dataclass(kw_only=True)
class DualFollowerConfig(RobotConfig):
    # ---------- Galaxea A1Z (can0, gs_usb) ----------
    galaxea_can_channel: str = "can0"
    galaxea_id: str = "follower1"           # 标定文件 robots/galaxea_a1z_follower/<id>.json
    galaxea_flip: str = "1,5,6"             # 1-based 反向关节 → joint_sign;本机确认 [-1,1,1,1,-1,-1]
    galaxea_scale: float = 1.0
    galaxea_max_vel_deg_s: float = 90.0     # 关节速度上限,越大越跟手(别过大防甩臂)
    galaxea_max_step_deg: float = 3.0       # 每步步长硬上限(安全阀)
    galaxea_gripper_kp: float = 15.0        # 夹爪 kp(过大易过热错误12)
    galaxea_gripper_margin: float = 0.0     # 夹爪两端留白,0=夹到底

    # ---------- reBot B601-RS (can5, PCAN/socketcan) ----------
    rebot_can: str = "can5"
    rebot_id: str = "follower1"             # 标定文件 robots/seeed_b601_rs_follower/<id>.json
    rebot_scale: float = 1.0                # 1:1 增量(与 Galaxea 幅度一致);>1 放大,<1 缩小
    rebot_flip: str = ""                    # 单验已确认全对, 一般空
    rebot_no_limit: bool = True             # True=不限速(MIT kp/kd 平滑, home锚点起步无跳变)
    rebot_max_step_deg: float = 3.0         # rebot_no_limit=False 时的每步限速

    # ---------- reBot 夹爪(直驱 7 号电机)----------
    grip_follow: bool = True
    grip_kp: float = 9.0
    grip_kd: float = 0.3
    grip_clamp_deg: float = 25.0            # 闭合端过冲(持续夹持力)
    grip_ratio_min: float = 0.62           # StarAI 夹爪比值到底约 0.6,从这里起算满行程
    grip_ratio_max: float = 1.0
    grip_max_step_deg: float = 25.0

    # ---------- 退出行为 ----------
    return_home_on_exit: bool = True        # reBot 回启动 home 姿态; Galaxea 走自身 return_to_zero

    # ---------- 相机(可选,官方 --display_data 自动进 rerun / 录进数据集)----------
    #   --robot.cameras='{ galaxea_wrist: {type: orbbec, serial_number_or_name: "CV2856D0006R", ...},
    #                       rebot_wrist:   {type: orbbec, serial_number_or_name: "CV275610002L", ...} }'
    #   USB2 链路务必 color_format: "mjpg"(未压缩会撑爆带宽导致取帧超时)。
    cameras: dict[str, CameraConfig] = field(default_factory=dict)

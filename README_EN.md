# Galaxea / reBot × StarAI Teleop — one leader, two follower arms (LeRobot)

Drive **two follower arms simultaneously (mirrored)** from a single **StarAI Violin** leader (7 servos incl. gripper):

- **Galaxea A1Z** — 6 joints + gripper, CAN, MIT force control
- **Seeed reBot B601-RS** — 6 joints + gripper, RobStride direct-drive motors, PCAN-USB / SocketCAN

Built on **LeRobot**. Includes LeRobot plugins/mapping for both arms, two out-of-tree CAN kernel drivers
(gs_usb / peak_usb), calibration, range matching, gripper follow + clamp, return-to-zero on exit, and an
optional **Orbbec depth camera** (RGB + aligned depth → rerun / dataset).

> Platform: NVIDIA Jetson (aarch64, kernel 6.8.12-tegra) · Leader UART@1M (CH340, `/dev/ttyCH341USB0`)
> · Galaxea CAN@1M (HHS USB-CANFD `a8fa:8598` → gs_usb, `can0`) · reBot CAN@1M (PCAN-USB `0c72:000c` → peak_usb, `can5`)

## Quick start

Prereq: a conda env with lerobot installed (default name `lerobot`).

```bash
git clone https://github.com/miracle-techlink/Galaxea_rebot_starai_tele && cd Galaxea_rebot_starai_tele

# 1) Install deps + both-arm SDKs/plugins + reBot official pip plugins + gs_usb/peak_usb + (optional) Orbbec
bash setup.sh                       # Galaxea only: WITH_REBOT=0   no camera: WITH_ORBBEC=0

# 2) Calibrate (follow prompts; the two arms' zero poses must physically correspond)
bash scripts/calibrate.sh           # Galaxea follower + StarAI leader
lerobot-calibrate --robot.type=seeed_b601_rs_follower --robot.port=can5 \
    --robot.can_adapter=socketcan --robot.id=follower1     # reBot zero
python scripts/teleop_starai_to_rebot.py --sweep          # reBot joint/gripper range sweep

# 3) Teleoperate
bash scripts/teleop.sh                                                        # Galaxea only
python scripts/teleop_starai_to_rebot.py --go --match-range --no-limit --grip-ratio-min 0.62   # reBot only
bash scripts/teleop_both.sh                                                   # both arms, mirrored
```

> ⚠️ Safety: fix both arms firmly, keep 1 m clearance around each, hand on e-stop. With `--no-limit` / at
> startup, first move the leader near both followers' current pose (near zero) to avoid a fast snap. Ctrl-C
> smoothly returns both arms to zero.

## Layout & docs

- `scripts/` — teleop bridges (`teleop_starai_to_rebot.py`, `teleop_starai_to_both.py`, `teleop_starai_to_a1z.py`), `.sh` wrappers, `rebot_follower_range.json`.
- `lerobot_plugins/` — `galaxea_a1z_follower`, `starai_violin_leader`, `cameras/orbbec` + install scripts.
- `gs_usb_module/`, `peak_usb_module/` — out-of-tree CAN kernel drivers (rebuilt against the running kernel).
- `third_party/` — GALAXEA-A1Z / fashionstar uservo SDKs (their own licenses).
- `docs/TELEOP_LEROBOT.md` (mapping/calibration/safety/reBot/camera/CAN) · `docs/SETUP_LOG.md` (hardware/driver debugging). Chinese `README.md` is the primary doc.

reBot's RobStride follower + 102 leader use the **official pip plugins** (`lerobot-robot-seeed-b601`,
`lerobot-teleoperator-rebot-arm-102`, installed by setup.sh), auto-registered via entry points.

## Licensing
Original code (`lerobot_plugins/`, `scripts/`) is Apache-2.0 (`LICENSE`). `gs_usb_module` / `peak_usb_module`
derive from the Linux kernel CAN drivers (GPL-2.0). `third_party/` retains upstream licenses.

# Galaxea / reBot × StarAI Teleop — one leader driving two arms (LeRobot)

[中文](README.md) · [![License](https://img.shields.io/badge/license-Apache--2.0-blue)](LICENSE)

One **StarAI Violin** leader (7 servos incl. gripper) **mirrors two follower arms at once**:

- **Galaxea A1Z** (6 joints + gripper, CAN, MIT force control)
- **Seeed reBot B601-RS** (6 joints + gripper, RobStride direct-drive, PCAN-USB / SocketCAN)

Packaged as a standard **LeRobot `Robot` plugin `dual_follower`** — drive it with the stock
**`lerobot-teleoperate` / `lerobot-record`** CLIs. Ships the follower mappings, two self-built CAN kernel
drivers (gs_usb / peak_usb), calibration, gripper follow+hold, auto-home on exit, and optional
**Orbbec depth cameras** (RGB → rerun / dataset).

> Platform: NVIDIA Jetson (aarch64, kernel 6.8.12-tegra) · leader UART@1M (CH340, `/dev/ttyCH341USB0`)
> · Galaxea CAN@1M (HHS USB-CANFD `a8fa:8598` → gs_usb, `can0` here) · reBot CAN@1M (PCAN-USB `0c72:000c` → peak_usb, `can5` here)

---

## Quick start

Prereq: a conda env with lerobot installed (default name `lerobot`).

```bash
git clone https://github.com/miracle-techlink/Galaxea_rebot_starai_tele && cd Galaxea_rebot_starai_tele

# 1) One-shot install: deps + arm SDKs/plugins (incl. dual_follower) + reBot official plugins + gs_usb/peak_usb + (optional) Orbbec
bash setup.sh
#   Galaxea only: WITH_REBOT=0 bash setup.sh    no cameras: WITH_ORBBEC=0 bash setup.sh

# 2) Bring up CAN
bash scripts/setup_follower_can.sh                          # can0=Galaxea(gs_usb), can5=reBot(PCAN)

# 3) Calibrate all three (dual_follower needs BOTH followers calibrated)
lerobot-calibrate --robot.type=galaxea_a1z_follower   --robot.can_channel=can0 --robot.id=follower1
lerobot-calibrate --robot.type=seeed_b601_rs_follower --robot.port=can5 --robot.can_adapter=socketcan --robot.id=follower1
lerobot-calibrate --teleop.type=starai_violin_leader  --teleop.port=/dev/ttyCH341USB0 --teleop.id=leader1

# 4) Dual-arm teleop (stock CLI, recommended) — spawns rerun (both wrist cams + joints); Ctrl-C to stop, arms auto-home
conda activate lerobot
bash scripts/teleop_dual.sh
#   no cameras: NO_CAM=1 bash scripts/teleop_dual.sh
#   pass through CLI args: bash scripts/teleop_dual.sh --robot.rebot_scale=1.0 --fps=30

# 5) Record a dataset (stock lerobot-record)
REPO_ID=you/dual_pick TASK="pick the cube" EPISODES=20 bash scripts/record_dual.sh
```

> ⚠️ **Safety**: bolt both arms down, keep 1 m clearance each, hand on e-stop. Thanks to the **home-anchored map**
> (below), at `--go` the follower target equals its current pose — **no startup snap**, so `--no-limit` is safe.
> On Ctrl-C: reBot ramps back to its startup home, Galaxea returns to zero.

<details>
<summary>No stock CLI? Use the equivalent argparse bridge scripts (legacy shortcut)</summary>

```bash
# single-arm quick checks
bash scripts/teleop.sh                                                     # Galaxea only
python scripts/teleop_starai_to_rebot.py --go --no-limit --grip-ratio-min 0.62   # reBot only
# dual (equivalent to dual_follower, includes the home-anchor fix)
python scripts/teleop_starai_to_both.py --arms both --go --galaxea-flip 1,5,6 \
    --grip-ratio-min 0.62 --no-limit --display-data \
    --galaxea-cam CV2856D0006R --rebot-cam CV275610002L --cam-format mjpg
```
</details>

---

## The `dual_follower` plugin

`lerobot_plugins/robots/dual_follower/` — a standard LeRobot `Robot`, `type: dual_follower`. It builds both
`GalaxeaA1ZFollower` + `SeeedB601RSFollower` internally, fans the leader action out to both in `send_action`,
and `get_observation` merges both arms' joints (`galaxea_* / rebot_*`) + both wrist cameras — so
`lerobot-record` captures it as a dataset directly.

### Mapping: home-anchored 1:1 delta (the key fix)

Galaxea's `leader_deg` semantics are `target = own_home + sign·scale·leader_deg` (1:1 delta from each arm's
calibrated home). **The old reBot map `target = gain·leader_deg` (zero-anchored, no offset) was wrong**:

- wrong magnitude vs Galaxea (gain≠1) → the two arms' trajectories don't match;
- reBot's one-sided joints (`shoulder_lift[0,170]` / `elbow_flex[0,200]`) clamp to 0 on any negative leader angle → **dead zones**.

Now reBot is anchored on **its own startup pose** too:

```
rebot_target[i] = rb_home[i] + sign[i]·scale·(leader[i] − leader_home[i])
```

`rb_home` = `rebot.get_observation()` at connect (abs degrees), `leader_home` = first leader read. Default
`scale=1` → same 1:1 delta as Galaxea, matched magnitude, **no dead zone, no startup snap**. On exit reBot
returns to `rb_home` (not 0 — reBot's 0 is a joint limit for shoulder_lift).

### Common params (`--robot.<key>`)

| Param | Default | Meaning |
|---|---|---|
| `galaxea_flip` | `1,5,6` | Galaxea inverted joints → `joint_sign=[-1,1,1,1,-1,-1]` (measured) |
| `galaxea_max_vel_deg_s` | `90` | Galaxea joint speed cap; raise if sluggish (120–150) |
| `rebot_scale` | `1.0` | reBot delta gain, 1 = matches Galaxea |
| `rebot_no_limit` | `true` | reBot unclamped (MIT kp/kd smoothing; home-anchor = no startup jump) |
| `grip_ratio_min` | `0.62` | StarAI gripper bottoms at ratio≈0.6 not 0; remap to full follower range |
| `grip_clamp_deg` | `25` | reBot gripper close-side overdrive (holding force) |
| `grip_kp` / `grip_kd` | `9 / 0.3` | reBot gripper (motor 7) MIT stiffness/damping |
| `return_home_on_exit` | `true` | auto-home both arms on exit |

---

## Directory layout

```
.
├── setup.sh                         # one-shot install (deps/SDK/plugins/CAN drivers/cameras)
├── scripts/
│   ├── setup_follower_can.sh        # bring up can0(Galaxea) / can5(reBot)
│   ├── teleop_dual.sh               # ★ stock lerobot-teleoperate wrapper (dual_follower + 2 cams + rerun)
│   ├── record_dual.sh               # ★ stock lerobot-record wrapper (dual-arm dataset)
│   ├── teleop.sh                    # Galaxea-only teleop (stock CLI)
│   ├── teleop_starai_to_both.py     # equivalent dual bridge (legacy shortcut, incl. home-anchor fix)
│   ├── teleop_starai_to_rebot.py    # StarAI→reBot bridge (mapping/gripper/home/--sweep/--go)
│   ├── teleop_starai_to_a1z.py      # StarAI→Galaxea passthrough (dry-run direction check)
│   ├── usbreset_orbbec.py           # soft-reset USB2 camera pipeline when it hangs (no replug)
│   ├── rebot_follower_range.json    # reBot measured joint+gripper ranges (--sweep output, sample)
│   └── scan_leader_starai.py
├── lerobot_plugins/                 # ★ custom LeRobot plugins
│   ├── robots/dual_follower/                # ★ composite follower (Galaxea+reBot, home-anchor map)
│   ├── robots/galaxea_a1z_follower/         # Galaxea follower (CAN + gripper + safety + optional cam)
│   ├── teleoperators/starai_violin_leader/  # StarAI leader (UART)
│   ├── cameras/orbbec/                       # Orbbec depth camera (RGB + aligned depth)
│   ├── install.sh                            # install galaxea+dual_follower+starai plugins & register
│   └── install_orbbec.sh                     # optional: orbbec camera plugin (+ small lerobot core patch)
├── gs_usb_module/                   # self-built gs_usb driver (Galaxea's HHS adapter)
├── peak_usb_module/                 # self-built peak_usb driver (reBot's PCAN-USB)
├── third_party/                     # upstream SDKs (GALAXEA-A1Z / fashionstar uservo, see LICENSEs)
└── docs/
    ├── TELEOP_LEROBOT.md            # mapping/calibration/safety/reBot/cameras/CAN details
    └── SETUP_LOG.md                 # full hardware/driver troubleshooting log
```

> reBot's RobStride follower + 102 leader use the **official pip plugins** (`lerobot-robot-seeed-b601` /
> `lerobot-teleoperator-rebot-arm-102`, installed by setup.sh) via entry-point auto-registration;
> `dual_follower` lazy-imports it in `__init__`.

---

## The two followers

| | Galaxea A1Z | Seeed reBot B601-RS |
|---|---|---|
| Motors | MIT force control (a1z SDK) | RobStride direct-drive (motorbridge) |
| CAN | HHS USB-CANFD → **gs_usb** → `can0` | PCAN-USB → **peak_usb** → `can5` |
| lerobot type | `galaxea_a1z_follower` (this repo) | `seeed_b601_rs_follower` (official pip) |
| Mapping | built-in `leader_deg` (home+delta) | `dual_follower` home-anchored 1:1 delta |
| Direction (measured) | `joint_sign=[-1,1,1,1,-1,-1]` (`galaxea_flip=1,5,6`) | all correct, no flip |
| Gripper | follower's built-in ratio map | `dual_follower` **direct-drives motor 7** (official gripper is stubbed) + hold overdrive + ratio remap |

---

## CAN interfaces (numbers shift on USB re-enumeration!)

```bash
# which driver/device each canX maps to
for i in $(seq 0 5); do echo -n "can$i: "; basename $(readlink -f /sys/class/net/can$i/device/driver 2>/dev/null) 2>/dev/null; done
# bring up
sudo ip link set can0 up type can bitrate 1000000 restart-ms 100   # Galaxea (gs_usb)
sudo ip link set can5 up type can bitrate 1000000 restart-ms 100   # reBot (PCAN)
```
Neither kernel driver ships in the Tegra kernel; `setup.sh` builds both from mainline source against the running kernel headers (see `*_module/`).

---

## FAQ (details in docs/)

- **reBot trajectory doesn't match leader/Galaxea**: make sure you're on the new map (`dual_follower` or the
  fixed `teleop_starai_to_both.py`); **do NOT** add `--match-range` — it's home-anchored 1:1 now, that flag re-introduces mismatch.
- **A reBot joint won't move**: usually a RobStride fault →
  `motorbridge-cli run --vendor robstride --channel can5 --model rs-00 --motor-id N --feedback-id 0xFD --mode clear-error --loop 1`
- **reBot gripper weak/sluggish**: raise `--robot.grip_clamp_deg` / `--robot.grip_kp`; tune `--robot.grip_ratio_min` if it doesn't fully close.
- **Every Galaxea joint offset**: usually the leader `leader1` was re-zeroed → re-zero Galaxea to match; `joint_sign` is a runtime param (not in the calibration file) — don't lose it.
- **Galaxea sluggish**: raise `--robot.galaxea_max_vel_deg_s` (default 90, up to 120–150).
- **Camera 8s capture timeout / `failed to capture frames`**: Gemini 305 uncompressed stream saturates **USB2**
  bandwidth — **use `color_format: mjpg`** (wrappers default to it); after repeated start/stop the pipeline may
  hang, run `python scripts/usbreset_orbbec.py` to soft-reset, ideally use a USB3 port.
- **`Failed to find Rerun Viewer executable in PATH`**: the rerun viewer binary sits next to the env python;
  wrappers add it to `PATH` automatically. For raw CLI: `export PATH=$(dirname $(which python)):$PATH`.

## Maintenance

```bash
cd ~/Galaxea_rebot_starai_tele
bash sync.sh                                   # sync from ~/ROBOT ARM + lerobot src (strips hardcoded paths)
git add -A && git commit -m "update: ..."
bash push.sh                                   # push straight to github (bypass the read-only ghfast mirror)
```

## License / credits
- `lerobot_plugins/` original to this project (LeRobot Apache-2.0 style).
- `gs_usb_module` / `peak_usb_module` based on Linux kernel CAN drivers (GPL-2.0) — gs_usb adds HHS `a8fa:8598` support + bulk-OUT endpoint fix; peak_usb rebuilt for the Tegra kernel.
- `third_party/`: [GALAXEA-A1Z](https://github.com/userguide-galaxea/GALAXEA-A1Z) · [fashionstar-uart-servo-python](https://github.com/servodevelop/fashionstar-uart-servo-python) (© original authors).
- reBot official: [reBot-DevArm](https://github.com/Seeed-Projects/reBot-DevArm) · [wiki](https://wiki.seeedstudio.com/rebot_arm_b601_rs_lerobot/).

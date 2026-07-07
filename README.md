# Galaxea / reBot × StarAI Teleop — 一主同驱双臂遥操作(LeRobot)

[English](README_EN.md) · [![License](https://img.shields.io/badge/license-Apache--2.0-blue)](LICENSE)

用一支 **StarAI Violin** 主臂(7 舵机含夹爪),**同时镜像遥操作两条从臂**:

- **星海图 Galaxea A1Z**(6 关节 + 夹爪,CAN,MIT 力控)
- **Seeed reBot B601-RS**(6 关节 + 夹爪,RobStride 直驱电机,PCAN-USB / SocketCAN)

已封装成一个标准 **LeRobot `Robot` 插件 `dual_follower`**,可直接用官方 **`lerobot-teleoperate` / `lerobot-record`** CLI
驱动与采集数据集。含两条从臂的映射、两套自编译 CAN 内核驱动(gs_usb / peak_usb)、标定、夹爪跟随+夹持、
退出自动归位,以及可选 **Orbbec 深度相机**(RGB → rerun / 数据集)。

> 平台:NVIDIA Jetson(aarch64,内核 6.8.12-tegra)· 主臂 UART@1M(CH340,`/dev/ttyCH341USB0`)
> · Galaxea CAN@1M(HHS USB-CANFD `a8fa:8598` → gs_usb,本机 `can0`)· reBot CAN@1M(PCAN-USB `0c72:000c` → peak_usb,本机 `can5`)

---

## 快速开始

前提:已有一个装好 lerobot 的 conda 环境(默认名 `lerobot`)。

```bash
git clone https://github.com/miracle-techlink/Galaxea_rebot_starai_tele && cd Galaxea_rebot_starai_tele

# ① 一键安装:依赖 + 两臂SDK/插件(含 dual_follower)+ reBot官方插件 + gs_usb/peak_usb驱动 + (可选)Orbbec相机
bash setup.sh
#   只装 Galaxea: WITH_REBOT=0 bash setup.sh    不装相机: WITH_ORBBEC=0 bash setup.sh

# ② 起 CAN 总线
bash scripts/setup_follower_can.sh                          # can0=Galaxea(gs_usb), can5=reBot(PCAN)

# ③ 标定(三个都要;dual_follower 依赖两条从臂各自的标定)
lerobot-calibrate --robot.type=galaxea_a1z_follower   --robot.can_channel=can0 --robot.id=follower1
lerobot-calibrate --robot.type=seeed_b601_rs_follower --robot.port=can5 --robot.can_adapter=socketcan --robot.id=follower1
lerobot-calibrate --teleop.type=starai_violin_leader  --teleop.port=/dev/ttyCH341USB0 --teleop.id=leader1

# ④ 双臂遥操作(官方 CLI,推荐)——弹 rerun 显示双腕相机+关节;Ctrl-C 停,两臂自动归位
conda activate lerobot
bash scripts/teleop_dual.sh
#   不要相机: NO_CAM=1 bash scripts/teleop_dual.sh
#   透传官方参数: bash scripts/teleop_dual.sh --robot.rebot_scale=1.0 --fps=30

# ⑤ 采集数据集(官方 lerobot-record)
REPO_ID=你的用户名/dual_pick TASK="pick the cube" EPISODES=20 bash scripts/record_dual.sh
```

> ⚠️ **安全**:两臂各自固定牢、各留 1 米净空、手放急停。得益于 **home 锚点映射**(下详),启 `--go` 瞬间
> 从臂目标 = 自身当前姿态,**起步不跳变**,`--no-limit` 也安全。Ctrl-C 退出:reBot 平滑回启动 home,Galaxea 回零。

<details>
<summary>没有官方 CLI?也可用等价的 argparse 桥接脚本(legacy 快捷入口)</summary>

```bash
# 单臂快速验证
bash scripts/teleop.sh                                                     # 仅 Galaxea
python scripts/teleop_starai_to_rebot.py --go --no-limit --grip-ratio-min 0.62   # 仅 reBot
# 双臂(等价 dual_follower,含 home 锚点映射修复)
python scripts/teleop_starai_to_both.py --arms both --go --galaxea-flip 1,5,6 \
    --grip-ratio-min 0.62 --no-limit --display-data \
    --galaxea-cam CV2856D0006R --rebot-cam CV275610002L --cam-format mjpg
```
</details>

---

## `dual_follower` 组合插件

`lerobot_plugins/robots/dual_follower/` —— 一个标准 LeRobot `Robot`,`type: dual_follower`。内部同时构造
`GalaxeaA1ZFollower` + `SeeedB601RSFollower`,在 `send_action` 里把主臂动作**同时分发给两条从臂**;
`get_observation` 汇总两臂关节(`galaxea_* / rebot_*`)+ 双腕相机,可直接被 `lerobot-record` 录成数据集。

### 映射:home 锚点 1:1 增量(关键修复)

Galaxea 的 `leader_deg` 语义是 `目标 = 自身home + sign·scale·leader角度`(从各自标定 home 出发、1:1 增量)。
**旧版 reBot 映射 `目标 = gain·leader角度`(零位对齐、无偏移)是错的**:

- 幅度与 Galaxea 不一致(gain≠1),两臂轨迹对不上;
- reBot 单边关节(`shoulder_lift[0,170]` / `elbow_flex[0,200]`)一遇主臂负角就被夹到 0,**大片死区**。

现在 reBot 也锚在**自己启动时的姿态**上:

```
reBot目标[i] = rb_home[i] + sign[i]·scale·(leader角度[i] − leader起始[i])
```

`rb_home` = 连接时 `rebot.get_observation()`(绝对度),`leader起始` = 首帧主臂读数。默认 `scale=1` → 与 Galaxea 同款
1:1 增量、幅度一致、**无死区、启动无跳变**。退出时 reBot 平滑回 `rb_home`(不是回 0,因为 reBot 的 0 对 shoulder_lift 是限位)。

### 常用参数(`--robot.<key>`)

| 参数 | 默认 | 说明 |
|---|---|---|
| `galaxea_flip` | `1,5,6` | Galaxea 反向关节 → `joint_sign=[-1,1,1,1,-1,-1]`(本机实测) |
| `galaxea_max_vel_deg_s` | `90` | Galaxea 关节速度上限,慢就调大(120~150) |
| `rebot_scale` | `1.0` | reBot 增量倍率,1=与 Galaxea 一致 |
| `rebot_no_limit` | `true` | reBot 不限速(MIT kp/kd 平滑;home 锚点起步无跳变) |
| `grip_ratio_min` | `0.62` | StarAI 夹爪捏到底 ratio≈0.6 不到 0,从这里起算满行程 |
| `grip_clamp_deg` | `25` | reBot 夹爪闭合端过冲(持续夹持力) |
| `grip_kp` / `grip_kd` | `9 / 0.3` | reBot 夹爪 7 号电机 MIT 刚度/阻尼 |
| `return_home_on_exit` | `true` | 退出两臂自动归位 |

---

## 目录结构

```
.
├── setup.sh                         # 一键安装(依赖/SDK/插件/CAN驱动/相机)
├── scripts/
│   ├── setup_follower_can.sh        # 起 can0(Galaxea)/ can5(reBot)总线
│   ├── teleop_dual.sh               # ★官方 lerobot-teleoperate 封装(dual_follower + 双相机 + rerun)
│   ├── record_dual.sh               # ★官方 lerobot-record 封装(采双臂数据集)
│   ├── teleop.sh                    # 仅 Galaxea 遥操作(标准 CLI)
│   ├── teleop_starai_to_both.py     # 等价双臂 argparse 桥接(legacy 快捷入口,含 home 锚点修复)
│   ├── teleop_starai_to_rebot.py    # StarAI→reBot 桥接(映射/夹爪/回零/--sweep/--go)
│   ├── teleop_starai_to_a1z.py      # StarAI→Galaxea 直通桥接(dry-run 验方向)
│   ├── usbreset_orbbec.py           # USB2 相机管线挂起时软复位(免拔插)
│   ├── rebot_follower_range.json    # reBot 各关节+夹爪实测行程(--sweep 产物,示例)
│   └── scan_leader_starai.py
├── lerobot_plugins/                 # ★自定义 LeRobot 插件
│   ├── robots/dual_follower/                # ★组合从臂(Galaxea+reBot,home锚点映射,type: dual_follower)
│   ├── robots/galaxea_a1z_follower/         # Galaxea 从臂(CAN+夹爪+安全+可选相机)
│   ├── teleoperators/starai_violin_leader/  # StarAI 主臂(UART)
│   ├── cameras/orbbec/                       # Orbbec 深度相机(RGB+对齐深度)
│   ├── install.sh                            # 装 galaxea+dual_follower+starai 插件并注册
│   └── install_orbbec.sh                     # 可选:装 orbbec 相机插件(含 lerobot 核心小改)
├── gs_usb_module/                   # 自编译 gs_usb 驱动(Galaxea 的 HHS 适配器)
├── peak_usb_module/                 # 自编译 peak_usb 驱动(reBot 的 PCAN-USB)
├── third_party/                     # 上游 SDK(GALAXEA-A1Z / fashionstar uservo,见各 LICENSE)
└── docs/
    ├── TELEOP_LEROBOT.md            # 映射/标定/安全/reBot/相机/CAN 详解
    └── SETUP_LOG.md                 # 硬件/驱动排障全过程
```

> reBot 的 RobStride 从臂 + 102 主臂用**官方 pip 插件**(`lerobot-robot-seeed-b601` / `lerobot-teleoperator-rebot-arm-102`,setup.sh 自动装),靠 entry-point 自动注册;`dual_follower` 在 `__init__` 里懒加载它。

---

## 两条从臂对照

| | Galaxea A1Z | Seeed reBot B601-RS |
|---|---|---|
| 电机 | MIT 力控(a1z SDK) | RobStride 直驱(motorbridge) |
| CAN | HHS USB-CANFD → **gs_usb** → `can0` | PCAN-USB → **peak_usb** → `can5` |
| lerobot type | `galaxea_a1z_follower`(本仓插件) | `seeed_b601_rs_follower`(官方 pip 插件) |
| 映射 | 内置 `leader_deg`(home+增量) | `dual_follower` 做 home 锚点 1:1 增量 |
| 方向(实测) | `joint_sign=[-1,1,1,1,-1,-1]`(`galaxea_flip=1,5,6`) | 全对,无需翻转 |
| 夹爪 | follower 内置按比例映射 | `dual_follower` **直驱 7 号电机**(官方插件夹爪是残的)+ 夹持过冲 + ratio 重映射 |

两条从臂合成一个 `dual_follower`,读一次主臂 → 同时喂两臂;标定各走各的 type。

---

## CAN 接口(号会随 USB 重枚举变化!)

```bash
# 看每个 canX 对应哪个驱动/设备
for i in $(seq 0 5); do echo -n "can$i: "; basename $(readlink -f /sys/class/net/can$i/device/driver 2>/dev/null) 2>/dev/null; done
# 起总线
sudo ip link set can0 up type can bitrate 1000000 restart-ms 100   # Galaxea (gs_usb)
sudo ip link set can5 up type can bitrate 1000000 restart-ms 100   # reBot (PCAN)
```
两个内核驱动在 Tegra 内核默认都没编,`setup.sh` 会用当前内核头文件**从 mainline 源码现编**(见 `*_module/`)。

---

## 常见问题(详见 docs/)

- **reBot 轨迹与主臂/Galaxea 对不上**:确认走的是新映射(`dual_follower` 或修复后的 `teleop_starai_to_both.py`);
  **不要**再加 `--match-range`——现在默认就是 home 锚点 1:1 增量,加了反而缩放不一致。
- **reBot 某关节不动**:多半 RobStride 卡故障 →
  `motorbridge-cli run --vendor robstride --channel can5 --model rs-00 --motor-id N --feedback-id 0xFD --mode clear-error --loop 1`
- **reBot 夹爪夹不紧/迟钝**:`--robot.grip_clamp_deg`(夹持过冲)/`--robot.grip_kp` 调大;捏到底不到位调 `--robot.grip_ratio_min`。
- **Galaxea 每个关节都偏**:多半重标过主臂 `leader1` 改了零位 → 重标 Galaxea 零位对齐;`joint_sign` 是运行时参数不在标定文件里,别丢。
- **Galaxea 慢**:`--robot.galaxea_max_vel_deg_s` 调大(默认 90,可到 120~150)。
- **相机取帧 8s 超时崩溃 / `failed to capture frames`**:Gemini 305 在 **USB2** 上未压缩流会撑爆带宽——
  **务必 `color_format: mjpg`**(封装脚本已默认);反复启停后管线可能挂起,跑 `python scripts/usbreset_orbbec.py` 软复位,最好插 USB3 口。
- **`Failed to find Rerun Viewer executable in PATH`**:rerun viewer 二进制与 env python 同目录,
  封装脚本已自动把它加进 `PATH`;手敲官方 CLI 时记得 `export PATH=$(dirname $(which python)):$PATH`。

## 维护 / 更新(本地开发)

```bash
cd ~/Galaxea_rebot_starai_tele
bash sync.sh                                   # 从 ~/ROBOT ARM + lerobot 源码同步进 repo(自动去硬编码路径)
git add -A && git commit -m "更新: ..."
bash push.sh                                   # 直连 github 推送(绕过全局 ghfast 只读镜像)
```

> 本机全局配了 `ghfast.top` 镜像加速(只读),普通 `git push` 会打到只读镜像失败;`push.sh` 已处理。

## 许可 / 来源
- `lerobot_plugins/` 本项目原创(LeRobot Apache-2.0 风格)。
- `gs_usb_module` / `peak_usb_module` 基于 Linux 内核 CAN 驱动(GPL-2.0)——gs_usb 加 HHS `a8fa:8598` 支持并修 bulk OUT 端点;peak_usb 为 Tegra 内核补编。
- `third_party/`:[GALAXEA-A1Z](https://github.com/userguide-galaxea/GALAXEA-A1Z) · [fashionstar-uart-servo-python](https://github.com/servodevelop/fashionstar-uart-servo-python)(版权归原作者)。
- reBot 官方:[reBot-DevArm](https://github.com/Seeed-Projects/reBot-DevArm) · [wiki](https://wiki.seeedstudio.com/rebot_arm_b601_rs_lerobot/)。

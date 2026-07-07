# Galaxea / reBot × StarAI Teleop — 一主同驱双臂遥操作(LeRobot)

用一支 **StarAI Violin** 主臂(7 舵机含夹爪),**同时镜像遥操作两条从臂**:

- **星海图 Galaxea A1Z**(6 关节 + 夹爪,CAN,MIT 力控)
- **Seeed reBot B601-RS**(6 关节 + 夹爪,RobStride 直驱电机,PCAN-USB / SocketCAN)

基于 **LeRobot**。含两条从臂的 LeRobot 插件/映射、两套自编译 CAN 内核驱动(gs_usb / peak_usb)、
标定、行程匹配、夹爪跟随+夹持、退出回零,以及可选 **Orbbec 深度相机**(RGB+对齐深度 → rerun/数据集)。

> 平台:NVIDIA Jetson(aarch64,内核 6.8.12-tegra)· 主臂 UART@1M(CH340,`/dev/ttyCH341USB0`)
> · Galaxea CAN@1M(HHS USB-CANFD `a8fa:8598` → gs_usb,本机 `can0`)· reBot CAN@1M(PCAN-USB `0c72:000c` → peak_usb,本机 `can5`)

---

## 快速开始

前提:已有一个装好 lerobot 的 conda 环境(默认名 `lerobot`)。

```bash
git clone https://github.com/miracle-techlink/Galaxea_rebot_starai_tele && cd Galaxea_rebot_starai_tele

# ① 一键安装:依赖 + 两臂SDK/插件 + reBot官方插件 + gs_usb/peak_usb驱动 + (可选)Orbbec相机
bash setup.sh
#   只装 Galaxea: WITH_REBOT=0 bash setup.sh    不装相机: WITH_ORBBEC=0 bash setup.sh

# ② 标定(按屏幕提示;两臂零位要物理对应)
bash scripts/calibrate.sh                                   # Galaxea 从臂 + StarAI 主臂
#   reBot 从臂零位标定:
lerobot-calibrate --robot.type=seeed_b601_rs_follower --robot.port=can5 \
    --robot.can_adapter=socketcan --robot.id=follower1
#   reBot 行程扫描(记录每关节+夹爪实际 min/max,给行程匹配/夹爪用;手扶着扫):
python scripts/teleop_starai_to_rebot.py --sweep

# ③ 遥操作
bash scripts/teleop.sh                                       # 仅 Galaxea
python scripts/teleop_starai_to_rebot.py --go --match-range --no-limit --grip-ratio-min 0.62   # 仅 reBot
bash scripts/teleop_both.sh                                  # ★双臂同步镜像
```

> ⚠️ **安全**:两臂各自固定牢、各留 1 米净空、手放急停。`--no-limit` / 起步时**先把主臂摆到两从臂当前接近的姿态**(都靠近零位),避免猛冲。Ctrl-C 退出会**自动平滑回零**。

---

## 目录结构

```
.
├── setup.sh                         # 一键安装(依赖/SDK/插件/CAN驱动/相机)
├── scripts/
│   ├── calibrate.sh                 # Galaxea + StarAI 标定
│   ├── teleop.sh                    # 仅 Galaxea 遥操作(标准 CLI)
│   ├── teleop_both.sh               # ★双臂同步遥操作(封装下面的 both 脚本)
│   ├── teleop_starai_to_rebot.py    # StarAI→reBot 桥接(映射/行程匹配/夹爪/回零/--sweep/--go)
│   ├── teleop_starai_to_both.py     # StarAI→Galaxea+reBot 双臂桥接
│   ├── teleop_starai_to_a1z.py      # StarAI→Galaxea 直通桥接(dry-run 验方向)
│   ├── rebot_follower_range.json    # reBot 各关节+夹爪实测行程(--sweep 产物,示例)
│   ├── setup_follower_can.sh / scan_leader_starai.py
├── lerobot_plugins/                 # ★自定义 LeRobot 插件
│   ├── robots/galaxea_a1z_follower/         # Galaxea 从臂(CAN+夹爪+安全+可选相机)
│   ├── teleoperators/starai_violin_leader/  # StarAI 主臂(UART)
│   ├── cameras/orbbec/                       # Orbbec 深度相机(RGB+对齐深度)
│   ├── install.sh                            # 装 galaxea+starai 插件并注册
│   └── install_orbbec.sh                     # 可选:装 orbbec 相机插件(含 lerobot 核心小改)
├── gs_usb_module/                   # 自编译 gs_usb 驱动(Galaxea 的 HHS 适配器)
├── peak_usb_module/                 # 自编译 peak_usb 驱动(reBot 的 PCAN-USB)
├── third_party/                     # 上游 SDK(GALAXEA-A1Z / fashionstar uservo,见各 LICENSE)
└── docs/
    ├── TELEOP_LEROBOT.md            # 映射/标定/安全/reBot/相机/CAN 详解
    └── SETUP_LOG.md                 # 硬件/驱动排障全过程
```

> reBot 的 RobStride 从臂 + 102 主臂用**官方 pip 插件**(`lerobot-robot-seeed-b601` / `lerobot-teleoperator-rebot-arm-102`,setup.sh 自动装),靠 entry-point 自动注册,不放进 `lerobot_plugins/`。

---

## 两条从臂对照

| | Galaxea A1Z | Seeed reBot B601-RS |
|---|---|---|
| 电机 | MIT 力控(a1z SDK) | RobStride 直驱(motorbridge) |
| CAN | HHS USB-CANFD → **gs_usb** → `can0` | PCAN-USB → **peak_usb** → `can5` |
| lerobot type | `galaxea_a1z_follower`(本仓插件) | `seeed_b601_rs_follower`(官方 pip 插件) |
| 映射 | 内置 `leader_deg`(follower 内做) | 桥接脚本做(键名/单位不同) |
| 方向(实测) | `joint_sign=[-1,1,1,1,-1,-1]`(`--galaxea-flip 1,5,6`) | 全对,无需翻转 |
| 夹爪 | follower 内置按比例映射 | 桥接**直驱 7 号电机**(官方插件夹爪是残的)+ 夹持过冲 + ratio 重映射 |

**双臂脚本关键参数**(`teleop_starai_to_both.py`,`teleop_both.sh` 已内置):
`--galaxea-flip 1,5,6 --galaxea-max-vel-deg-s 90`(放开 Galaxea 默认 50°/s 限速)、
`--match-range`(reBot 行程按 config 限位匹配)、`--no-limit`(reBot 不限速)、`--grip-ratio-min 0.62`
(StarAI 夹爪捏到底 ratio≈0.6 不到 0,重映射到从臂全行程)。

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

## 常见问题(详见 docs/）

- **reBot 某关节不动**:多半 RobStride 卡故障 →
  `motorbridge-cli run --vendor robstride --channel can5 --model rs-00 --motor-id N --feedback-id 0xFD --mode clear-error --loop 1`
- **reBot 夹爪夹不紧/合不拢**:`--grip-clamp-deg`(夹持过冲)/`--grip-kp` 调大;捏到底不到位调 `--grip-ratio-min`。
- **Galaxea 每个关节都偏**:多半重标过主臂 `leader1` 改了零位 → 重标 Galaxea 零位对齐;方向 `joint_sign` 不在标定文件里(是运行时参数,别丢,已记 docs)。
- **Galaxea 慢**:放开 `--galaxea-max-vel-deg-s`(默认 90,可到 120~150)。
- **相机 openUsbDevice failed**:装 Orbbec udev 规则(setup 会装);Gemini 305 带深度在 **USB2** 上不稳,建议插 USB3,或 `color_format=mjpg`。

## 许可 / 来源
- `lerobot_plugins/` 本项目原创(LeRobot Apache-2.0 风格)。
- `gs_usb_module` / `peak_usb_module` 基于 Linux 内核 CAN 驱动(GPL-2.0)——gs_usb 加 HHS `a8fa:8598` 支持并修 bulk OUT 端点;peak_usb 为 Tegra 内核补编。
- `third_party/`:[GALAXEA-A1Z](https://github.com/userguide-galaxea/GALAXEA-A1Z) · [fashionstar-uart-servo-python](https://github.com/servodevelop/fashionstar-uart-servo-python)(版权归原作者)。
- reBot 官方:[reBot-DevArm](https://github.com/Seeed-Projects/reBot-DevArm) · [wiki](https://wiki.seeedstudio.com/rebot_arm_b601_rs_lerobot/)。

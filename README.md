# lingbot-teleop — StarAI Violin → Galaxea A1Z 主从遥操作(LeRobot)

用 **StarAI Violin**(主臂,7 舵机含夹爪)遥操作 **星海图 Galaxea A1Z**(从臂,6 关节 + 夹爪),
基于 **LeRobot 0.4.4**,标准 `lerobot-teleoperate` CLI 直接驱动。含自编译 gs_usb 驱动、两臂
LeRobot 插件、标定、映射、安全功能,**面向一键启动**。

> 平台:NVIDIA Jetson (aarch64, 内核 6.8.12-tegra) · 主臂 UART@1M(CH340)· 从臂 CAN@1M(HHS USB-CANFD `a8fa:8598`)

---

## 一键启动(三步)

前提:已有一个装好 lerobot 的 conda 环境(默认名 `lerobot`;没有的话先按 LeRobot 官方装好)。

```bash
git clone <你的仓库地址> lingbot-teleop && cd lingbot-teleop

# ① 一键安装:依赖 + 两臂SDK + lerobot插件 + gs_usb驱动
bash setup.sh
#   可指定:CONDA_ENV=lerobot LEROBOT_SRC=/home/tommyzihao/lingbot/lerobot bash setup.sh

# ② 一键标定两臂(零位 + 各关节量程 + 夹爪开合;按屏幕提示)
bash scripts/calibrate.sh

# ③ 一键遥操作(手扶从臂、低速起)
bash scripts/teleop.sh
```

- 启动约 11 秒(torch/rerun import),不是卡死。
- **启动即移动 & Ctrl-C 会自动回零**:手和身体别挡在从臂运动路径上。
- CAN 接口自动探测(gs_usb 生成的 canX);主臂口默认 `/dev/ttyCH341USB0`。可用 `CAN_IF=` / `LEADER_PORT=` 覆盖。

---

## 目录结构

```
lingbot-teleop/
├── setup.sh                    # 一键安装(依赖/SDK/插件/驱动)
├── scripts/
│   ├── calibrate.sh            # 一键标定两臂
│   ├── teleop.sh               # 一键遥操作
│   ├── teleop_starai_to_a1z.py # 桥接脚本(直通 + dry-run 验方向)
│   ├── setup_follower_can.sh   # 单独拉起从臂 CAN
│   └── scan_leader_starai.py   # 单独扫主臂舵机
├── lerobot_plugins/            # ★核心自定义代码(LeRobot 插件)
│   ├── robots/galaxea_a1z_follower/        # 从臂 Robot(CAN + 夹爪 + 安全)
│   ├── teleoperators/starai_violin_leader/ # 主臂 Teleoperator(UART)
│   └── install.sh              # 装进 lerobot 源码树 + 注册
├── gs_usb_module/              # 自编译 gs_usb 驱动(修 EP + a8fa:8598 自动绑定)
│   ├── gs_usb.c / gs_usb.ko / Makefile
│   └── install.sh
├── third_party/                # 上游 SDK(见各自 LICENSE)
│   ├── GALAXEA-A1Z/                        # 从臂 a1z SDK(MIT)
│   └── fashionstar-uart-servo-python/      # 主臂 uservo SDK
└── docs/
    ├── TELEOP_LEROBOT.md       # 遥操作/映射/标定/安全 详解
    └── SETUP_LOG.md            # 硬件/gs_usb 驱动排障全过程
```

---

## 映射原理(简)

- 主臂 `get_action()`:臂关节输出"相对零位角度(度,带 ±180° 解绕)",夹爪输出"行程比例[0,1]"。
- 从臂 `send_action()`:臂关节 **1:1 直接角度**(`home + sign*deg2rad(主臂角)`)→ 夹标定范围/硬件限位 → 限速缓入;夹爪按比例映射到自己开合行程。
- 详见 `docs/TELEOP_LEROBOT.md`。

## 安全功能(从臂,均可调 `--robot.xxx=`)

| 功能 | 参数 | 默认 |
|---|---|---|
| 关节软限位(相对零位 ±°) | `max_joint_travel_deg` | null(关,用标定范围) |
| 最大跟随速度(°/s,防甩臂) | `max_joint_vel_deg_s` | 50 |
| 退出回零 + 闭合夹爪 | `return_to_zero_on_exit` / `return_speed_deg_s` | True / 25 |
| 关节方向 | `joint_sign` | `[-1,1,1,1,-1,-1]` |
| 夹爪 | `gripper_kp/gripper_max_step_rad/gripper_margin/gripper_sign` | 15 / 0.4 / 0.10 / -1 |

## 常见问题(详见 docs/)

- **从臂电机全无响应/总线静默**:查电机电源 + CAN 接头(尤其手动搬动后松动);`scan_leader_starai.py` / `motor_diag.py --scan` 排查。
- **CAN feedback stale → 急停**:gs_usb 在高频下偶发卡顿;已降到 150Hz;接头接触不良也会。
- **夹爪卡死(错误码12,线圈过热)**:kp 过高死顶硬限位所致;已用 kp=15 + margin=0.10。清错见 `docs/TELEOP_LEROBOT.md`。
- **方向反了**:改 `--robot.joint_sign` 对应位;夹爪开合反了改 `--robot.gripper_sign`。

## 说明
- `third_party/` 为上游开源 SDK,版权归原作者(见各目录 LICENSE):
  [GALAXEA-A1Z](https://github.com/userguide-galaxea/GALAXEA-A1Z) · [fashionstar-uart-servo-python](https://github.com/servodevelop/fashionstar-uart-servo-python)
- `gs_usb_module/gs_usb.c` 基于 Linux 内核 gs_usb 驱动(GPL-2.0)修改,加入 HHS `a8fa:8598` 支持并修正 bulk OUT 端点。
- lerobot 插件为本项目原创,按 LeRobot(Apache-2.0)风格编写。

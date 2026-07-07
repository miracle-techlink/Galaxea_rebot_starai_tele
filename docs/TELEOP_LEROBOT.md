# LeRobot 主从遥操作:StarAI Violin(主)→ Galaxea A1Z(从)

关节空间 **归一化范围映射**(同 SO-100 思路),标准 `lerobot-teleoperate` CLI 可直接驱动。
环境:`conda activate lerobot`(已并入两臂 SDK)。

---

## 1. 架构(LeRobot 0.4.4 插件)

在 lerobot 源码树(`/home/tommyzihao/lingbot/lerobot`)新增两个标准插件:

| 角色 | 类 | type | 位置 | 底层 |
|---|---|---|---|---|
| 主臂 leader | `StaraiViolinLeader(Teleoperator)` | `starai_violin_leader` | `src/lerobot/teleoperators/starai_violin_leader/` | uservo UART @1M, `/dev/ttyCH341USB0` |
| 从臂 follower | `GalaxeaA1ZFollower(Robot)` | `galaxea_a1z_follower` | `src/lerobot/robots/galaxea_a1z_follower/` | a1z SDK, CAN `can4` |

数据流(键 `joint_1.pos`…`joint_6.pos` + `gripper.pos`):
- 主臂 `get_action()` → **臂关节**输出"相对零位角度(度)"(读数带 ±180° 解绕);**夹爪**输出"行程比例 [0,1]"(其零位常在闭合端,故用比例而非居中)。
- 从臂 `send_action()`:**臂关节** 1:1 直接角度映射 → 夹标定范围+硬件限位 → 限速缓入;**夹爪** 按比例映射到自己开合行程。connect 即位置保持。
- **从臂有夹爪**(第 7 个 MotorB,CAN ID 0x07,SDK 未含,独立 socket 驱动)。

**臂映射公式**(1:1 直接角度,零位对齐 + 标定范围夹住):
```
target[i] = home_F[i] + sign[i]*scale*deg2rad(主臂相对角[i])
target[i] = clip(target[i], 标定range∩硬件限位[∩软限位])
```
→ 1 主臂度 ≈ 1 从臂度(响应直观、不压缩);越界自动夹住。
**夹爪映射**:主臂行程比例 [0,1] → 从臂开合行程([grip_min,grip_max] 内留 margin);`gripper_sign` 定开/闭对应端。

---

## 2. 标定(零位 + 限位,两臂都标)

每个臂两步:**① 摆到零位姿态按 Enter(记零位)→ ② 按 Enter 开始,把每个关节转到两端极限来回扫,再按 Enter(记范围)**。从臂标定时柔顺悬浮供手动摆/扫,扫描期间临时关限位急停(能扫到底)。

> 💡 两臂零位要**物理对应**(同一个你想对齐的构型)。零位处从臂各关节尽量居中。

```bash
conda activate lerobot
sudo ip link set can4 up type can bitrate 1000000

# ① 从臂:柔顺悬浮 → 摆零位 Enter → Enter 开始扫 → 各关节转到两端 → Enter 结束
lerobot-calibrate --robot.type=galaxea_a1z_follower --robot.can_channel=can4 --robot.id=follower1

# ② 主臂:摆零位 Enter → Enter 开始扫 → 各关节转到两端 → Enter 结束
lerobot-calibrate --teleop.type=starai_violin_leader --teleop.port=/dev/ttyCH341USB0 --teleop.id=leader1
```

存盘(按 `id` 命名,都含 home + range):
- 从臂 `.../robots/galaxea_a1z_follower/follower1.json` → `home_rad` / `range_min_rad` / `range_max_rad`
- 主臂 `.../teleoperators/starai_violin_leader/leader1.json` → `homing_offset_deg` / `range_min_deg` / `range_max_deg`

> ⚠️ `--id` 标定与遥操作必须一致。重标定先删对应 json。
> ⚠️ 每个关节都要真正转到两端极限,幅度比例才准。主臂读数带解绕,±180° 回绕不再污染范围。
> 从臂硬件限位(SDK,自动夹):joint1 ±120°、joint2 [0,180°]、joint3 [-180°,0°]、joint4/5 ±85°、joint6 ±115°。

---

## 3. 遥操作

### 方式 A:标准 lerobot-teleoperate CLI(推荐)
```bash
conda activate lerobot
sudo ip link set can4 up type can bitrate 1000000

lerobot-teleoperate \
    --robot.type=galaxea_a1z_follower --robot.can_channel=can4 --robot.id=follower1 \
    --robot.joint_sign='[-1,1,1,1,-1,-1]' \
    --teleop.type=starai_violin_leader --teleop.port=/dev/ttyCH341USB0 --teleop.id=leader1 \
    --fps=30
```
- ⏳ **启动约 11 秒**(torch/rerun import),不是卡死。
- ⚠️ **启动即移动**:从臂会限速移动到主臂当前姿态对应的映射位姿。**手扶、低速起**,不对 Ctrl-C。
- `--robot.joint_sign='[...]'`:6 关节方向(+1/-1),方向反的填 -1(先用 dry-run 定,见下)。
- 其他:`--robot.scale=1.0`(用多少比例的范围,保守可 0.7)、`--robot.max_step_rad=0.017`(≈1°/步,越小越慢越稳)、`--robot.control_freq_hz=250`、`--robot.gravity_comp_factor=1.0`。
- `--teleop_time_s=10` 限时;不加则 Ctrl-C 停。已实测退出码 0、~30Hz。

### 方式 B:桥接脚本(直通,便于验方向)
`/home/tommyzihao/ROBOT ARM/teleop_starai_to_a1z.py` —— `leader.get_action()`→`follower.send_action()` 直通,含安全 **dry-run**(从臂不动,只打印映射目标)。
```bash
# 方向验证(从臂不动):慢扳主臂各关节,看"从臂目标"往哪走,方向反的记下
python "/home/tommyzihao/ROBOT ARM/teleop_starai_to_a1z.py" --dry-run --flip 2,4
# 正式跟随
python "/home/tommyzihao/ROBOT ARM/teleop_starai_to_a1z.py" --flip 2,4
```
参数:`--dry-run` / `--flip 2,4`(翻转关节 1-6)/ `--scale` / `--max-step-deg 1.0` / `--freq 50` / `--can` / `--leader-port`。

### 方向标定(joint_sign)—— ✅ 已确认
先跑 dry-run 逐个扳主臂关节,确认从臂目标方向;反的关节记进 `--flip`(桥接)或 `--robot.joint_sign`(CLI)。
**✅ 实车确认(2026-07-07,基于当日重标的 leader1):Galaxea 翻转关节 1、5、6 → `joint_sign=[-1,1,1,1,-1,-1]`**
(桥接/双臂脚本用 `--galaxea-flip 1,5,6`)。
> ⚠️ joint_sign 不写进标定 json(是运行时参数),别再丢。重标主臂 `leader1` 会改零位/量程 → Galaxea 需重新对齐(重标 Galaxea 零位),方向一般不变。

---

## 3.7 视觉:奥比中光 Gemini 305 深度相机(RGB + 对齐深度 → rerun/数据集)

新增 lerobot 相机插件 **`orbbec`**(`src/lerobot/cameras/orbbec/`),基于 `pyorbbecsdk` v2,
输出 **彩色 `(H,W,3)` uint8 + 软件 D2C 对齐深度 `(H,W,1)` uint16(毫米)**。挂到从臂后,
`--display_data=true` 会把彩色 + 深度自动推进 **rerun**;`lerobot-record` 会存成数据集特征
`observation.images.<cam>`(彩色)与 `observation.images.<cam>_depth`(深度)。

**依赖(已装在 conda `lerobot` 环境)**:`pip install pyorbbecsdk2`(ARM64 预编译 wheel,
Jetson 官方支持);USB 权限用 `pyorbbecsdk/shared/install_udev_rules.sh`(sudo,已装到
`/etc/udev/rules.d/99-obsensor-libusb.rules`,Gemini 305 PID `0840` 在内,MODE 0666)。

**现场两台 Gemini 305(都是深度相机,一台装星海图腕部、一台装 Seeed reBot)**:
| 序列号 | 链路 | 说明 |
|---|---|---|
| `CV2856D0006R` | **USB3.2** | ✅ 1280×800@30 彩色+深度,connect~2s、30fps、disconnect 干净 |
| `CV275610002L` | ⚠️ **USB2.1** | 深度流在 USB2 上**不可靠**(裸抓能出帧,但线程化 pipeline 常在 open/stop 卡死)|

> ⚠️ **强烈建议两台都插 USB3 口**。Gemini 305 本是 USB3 相机,带深度引擎在 USB2 链路上
> 不稳:1280×800 RGB(未压缩~92MB/s)、乃至 640×480 RGB+深度(~45MB/s)都超 USB2 有效带宽(~35MB/s),
> 且实测线程化 pipeline 在 USB2 上 `start()/stop()` 会卡死、需 `reboot()` 或重插才能恢复。
> 移到 USB3 后即与另一台一样稳。若**必须**留在 USB2:设 `color_format: mjpg` 压缩彩色、深度会自动降到
> 848×530@10;插件的 `disconnect()` 已加 **stop() 看门狗**(3s 超时不阻塞 teleop 退出)。
> 卡死自救:`python -c "import pyorbbecsdk as ob; ctx=ob.Context(); dl=ctx.query_devices(); [dl.get_device_by_index(i).reboot() for i in range(dl.get_count())]"`

列出在线相机:`python -c "from lerobot.cameras.orbbec import OrbbecCamera; import json; print(json.dumps(OrbbecCamera.find_cameras(),indent=2,default=str))"`

**遥操作 + 腕部深度相机 + rerun**(把 `wrist` 的序列号换成你星海图腕部那台、优先 USB3):
```bash
conda activate lerobot
sudo ip link set can4 up type can bitrate 1000000

lerobot-teleoperate \
    --robot.type=galaxea_a1z_follower --robot.can_channel=can4 --robot.id=follower1 \
    --robot.joint_sign='[1,-1,1,-1,1,1]' \
    --robot.cameras='{ wrist: {type: orbbec, serial_number_or_name: "CV2856D0006R", fps: 30, width: 1280, height: 800, use_depth: true} }' \
    --teleop.type=starai_violin_leader --teleop.port=/dev/ttyCH341USB0 --teleop.id=leader1 \
    --fps=30 --display_data=true
```
- 相机在机器人里**必须**给 `fps/width/height`(lerobot `RobotConfig` 强制,用于确定特征形状)。
- 只要彩色不要深度:去掉 `use_depth: true`(或设 false)。
- 多相机:`'{ wrist: {...}, top: {...} }'`(注意 USB 总带宽/口)。
- 相机配置项(`OrbbecCameraConfig`):`serial_number_or_name`、`use_rgb`/`use_depth`、`align_to_color`(默认 True)、
  `color_format`(auto/mjpg/rgb/yuyv/bgr)、`color_mode`(RGB/BGR)、`rotation`、`fps/width/height`。

**插件改动清单**(更新 lerobot 会覆盖,和两臂插件同理):
- 新增 `src/lerobot/cameras/orbbec/`(`configuration_orbbec.py` + `camera_orbbec.py` + `__init__.py`)。
- `cameras/utils.py` 加 `elif cfg.type=="orbbec"` 分支;`utils/import_utils.py` 加 `_pyorbbecsdk_available`。
- 4 个脚本(teleoperate/record/calibrate/rollout)加 `from lerobot.cameras.orbbec import OrbbecCameraConfig  # noqa`(draccus 解析 `type: orbbec` 需在 parse 前注册)。
- `galaxea_a1z_follower`(config + robot)加 `cameras` 字段并接进 `get_observation`/`observation_features`/`connect`/`disconnect`。

> reBot B601_RS 从臂本身**已自带** `cameras` 字段,直接 `--robot.cameras='{...type: orbbec...}'` 即可。

---

## 3.5 安全功能(从臂,全部可调)

| 功能 | 参数 | 默认 | 说明 |
|---|---|---|---|
| ① 关节软限位 | `--robot.max_joint_travel_deg` | null(关) | 每个臂关节相对零位最多 ±此角度(度);默认关=用标定范围(能到极限);设数字(如60)才额外限制 |
| ② 最大跟随速度 | `--robot.max_joint_vel_deg_s` | 50 | 从臂关节最大速度(度/秒),防甩臂打人;按实测 dt 限速,`max_step_rad` 是硬backstop |
| ③ 退出回零 | `--robot.return_to_zero_on_exit` / `--robot.return_speed_deg_s` | True / 25 | Ctrl-C 结束时从臂**平滑回零位并闭合夹爪**,避免停在坏姿态 |

夹爪(B 组默认):`gripper_kp=15`、`gripper_max_step_rad=0.4`、`gripper_margin=0.10`、`gripper_sign=-1`。
> ⚠️ 夹爪 kp 过高 + margin 过小 → 死顶闭合硬限位 → 线圈过热(错误码12)卡死。清错:`MotorB(7).clear_error()`。
> ⚠️ **Ctrl-C 后从臂会自动移动回零位**(25°/s),手别挡着。

## 4. 环境与安装(如何搭起来的)

- **lerobot 源码**:`/home/tommyzihao/lingbot/lerobot`(v0.4.4,Seeed fork)。原 editable 指向已失效的 `/home/tommyzihao/lerobot`,已改 `.pth`(`.../envs/lerobot/lib/python3.12/site-packages/__editable__.lerobot-0.4.4.pth`)指向 `.../lingbot/lerobot/src`。
- **conda `lerobot` 环境**已并入两臂 SDK:
  ```bash
  PY=/home/tommyzihao/miniconda3/envs/lerobot/bin/python
  $PY -m pip install "python-can>=4.0"
  $PY -m pip install -e "/home/tommyzihao/ROBOT ARM/GALAXEA-A1Z" --no-deps
  conda install -n lerobot -c conda-forge pinocchio
  # uservo 通过 .pth: echo ".../fashionstar-uart-servo-python/src" > site-packages/uservo_src.pth
  ```
  已装:lerobot 0.4.4 / python-can / pinocchio 4.0 / a1z(editable)/ uservo(.pth)/ pyserial。
- **注册**:两个 type 通过 `robots/__init__.py`、`teleoperators/__init__.py` 里的 `from . import ...` 触发 `register_subclass`。工厂靠 `make_device_from_device_class` 约定实例化(config 模块名去掉 `config_`)。

> ⚠️ **更新 lerobot 会覆盖**这两处 `__init__` 改动 + 插件目录。更新后需重加,或改成外部插件包 `lerobot_robot_galaxea_a1z` / `lerobot_teleoperator_starai_violin`(pip 装后由官方 `register_third_party_plugins()` 自动加载,免覆盖)。

---

## 5. 演进与关键修复(踩过的坑)

1. **限速蠕动 bug**:初版把每步目标 clamp 到实测当前位置 → PD 误差恒极小 → 从臂只蠕动。改为维护**内部指令设定点**领先实际位置,PD 误差足够大能真正驱动。✅
2. **joint2 越界被拒**:早期用"零位偏置 + 1:1 度映射",零位又常在关节限位边缘(joint2 起点 0°=下限)→ 一动就越界被 SDK 拒。改为**归一化范围映射**(两臂标定范围,norm↔范围),永不越界。✅
3. **CLI 默认恒等 processor**:`lerobot-teleoperate` 不做单位/范围转换,直接把主臂输出丢给从臂 → 映射必须做进**从臂 `send_action`**,CLI 才可用。✅
4. **启动慢≠卡死**:CLI 启动 import 约 11 秒。
5. **偶发**:见过一次 MotorA `error_code=0x1F`+位置乱值触发急停(疑 gs_usb 偶发坏帧,未复现)。若频繁,降 `control_freq_hz` 或加坏帧容错。

## 6. 状态
- 主/从插件、标定(范围)、归一化映射、CLI 与桥接两种启动 —— 均实现并单元/离线验证 ✅
- 视觉:`orbbec` 相机插件(RGB+对齐深度)已实现,USB3 那台真机验证 connect/流/disconnect 全通过 ✅;
  USB2 那台建议移到 USB3(见 §3.7)。
- 待你实车:两臂范围标定 + dry-run 定方向 + 正式跟随联调 + 腕部相机联调(优先都插 USB3)。
- 相关文件:`teleop_starai_to_a1z.py`(桥接)、`SETUP_LOG.md`(硬件/gs_usb 驱动)、本文件。

---

## 7. Seeed reBot-DevArm B601_RS(另一套从臂,依赖已装)

现场硬件 = Seeed **reBot-DevArm B601_RS**(Damiao CAN 电机),对应 lerobot 插件
`rebot_b601_follower`(从臂,已自带 `cameras` 字段)/ `rebot_102_leader`(主臂)。
依赖已装进 conda `lerobot` 环境(官方 extra `lerobot[rebot]`):
- `motorbridge 0.3.9`(从臂 Damiao CAN,`--robot.can_adapter=damiao` 走 `/dev/ttyACM*` 串口桥,或 `socketcan`)
- `motorbridge-smart-servo 0.0.4`(reBot Arm 102 主臂 FashionStar 舵机)

> ⚠️ 装在 **conda `lerobot`** 环境(lerobot 源码 → `lingbot/lerobot/src`,活的)。
> `lingbot/lingbot-venv` 里的 lerobot editable 指向已失效的 `/home/tommyzihao/lerobot/src`,
> 在那个 venv 里 `import lerobot` 直接失败,**跑不了 reBot 插件**(那是 genesis/仿真 venv)。

> ⚠️ **重要**:现场是 **B601-RS(RobStride 电机)**,不是 B601-DM。lerobot 树里的
> `rebot_b601_follower` 是 **Damiao** 版,**驱动不了 RS**。用官方验证插件(见 §8)。

---

## 8. Seeed reBot B601-RS(RobStride)—— 官方插件 + StarAI 桥接

**电机 RobStride,走 PCAN-USB → SocketCAN,1Mbps。** 不是 Damiao、不是串口桥。

### 8.1 CAN 通道(peak_usb 本机需自编)
Jetson `6.8.12-tegra` 内核**没带 `peak_usb`**。已 out-of-tree 编好(同 gs_usb 套路,源码 mainline v6.8):
装在内核模块树 + 开机自动加载(`/etc/modules-load.d/peak_usb.conf`),备份 `~/ROBOT ARM/peak_usb_module/`。
**PCAN = `can5`**(can0 被 gs_usb/Galaxea 占了)。
```bash
sudo modprobe peak_usb
sudo ip link set can5 up type can bitrate 1000000 restart-ms 100
ip -br link show can5    # UP, state ERROR-ACTIVE
# 扫电机(只读): motorbridge-cli scan --vendor robstride --channel can5 --start-id 1 --end-id 7
```
> can 口号会随 USB 重枚举变化;`ls -l /sys/class/net/can*/device/driver` 看谁是 peak_usb / gs_usb。

### 8.2 官方 lerobot 插件(已装, `--no-deps`)
```bash
pip install lerobot-robot-seeed-b601 lerobot-teleoperator-rebot-arm-102
```
- `seeed_b601_rs_follower`(RobStride 从臂,socketcan,feedback 0xFD,MIT,自带标定)
- `rebot_arm_102_leader`(官方 Star Arm 102-L 主臂)
- 外部包,靠 `register_third_party_plugins()` 自动注册,免改源码树。

### 8.3 标定(零位:关力矩→手动摆零位+闭夹爪→回车)
```bash
conda activate lerobot   # CAN 先 up(§8.1)
lerobot-calibrate --robot.type=seeed_b601_rs_follower --robot.port=can5 \
    --robot.can_adapter=socketcan --robot.id=follower1
```
> 重标定先删 `~/.cache/huggingface/lerobot/calibration/{robots,teleoperators}` 下对应 json。

### 8.4 遥操作:StarAI 主臂 → reBot(桥接脚本)
StarAI 输出 `joint_1..6`,reBot 从臂要 `shoulder_pan..`(绝对度)——键/单位不同,标准 CLI 不通,用
`/home/tommyzihao/ROBOT ARM/teleop_starai_to_rebot.py`(默认 **dry-run**:从臂松力矩只打印目标验方向):
```bash
# 1) 方向验证(臂不动): 慢扳主臂, 看"从臂目标"方向, 反的记进 --flip
python "/home/tommyzihao/ROBOT ARM/teleop_starai_to_rebot.py"
# 2) 正式跟随(手扶/低速起/Ctrl-C 停): 小步长先
python "/home/tommyzihao/ROBOT ARM/teleop_starai_to_rebot.py" --go --flip 3,4,5 --max-step-deg 2 --freq 20
```
参数:`--flip`(翻方向)/`--scale`/`--max-step-deg`(每步度数上限=安全阀)/`--freq`/`--grip-open-deg`/`--grip-close-deg`。
> 官方一对一(reBot-102 主臂)则用标准 CLI:`lerobot-teleoperate --robot.type=seeed_b601_rs_follower
> --robot.port=can5 --robot.can_adapter=socketcan --robot.id=follower1 --teleop.type=rebot_arm_102_leader
> --teleop.port=/dev/ttyUSB0 --teleop.id=...`(相机同 §3.7)。

### 8.5 目标:一个 StarAI 同驱两臂(Galaxea+reBot)
待单臂各自验证后,写双臂桥接(读一次 StarAI → 同时映射发 Galaxea.send_action + reBot.send_action)。**先单验,再合双。**

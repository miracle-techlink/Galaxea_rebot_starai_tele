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

### 方向标定(joint_sign)
先跑 dry-run 逐个扳主臂关节,确认从臂目标方向;反的关节记进 `--flip`(桥接)或 `--robot.joint_sign`(CLI)。已初步验证 2、4 需反 → `[1,-1,1,-1,1,1]`,建议 6 个逐一确认。

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
- 待你实车:两臂范围标定 + dry-run 定方向 + 正式跟随联调。
- 相关文件:`teleop_starai_to_a1z.py`(桥接)、`SETUP_LOG.md`(硬件/gs_usb 驱动)、本文件。

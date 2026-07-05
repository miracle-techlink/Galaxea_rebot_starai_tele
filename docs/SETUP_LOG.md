# 双臂遥操作环境搭建日志

> 主臂(leader)= StarAI Violin(飞特 UART 舵机)· 从臂(follower)= 星海图 A1Z(CAN 力控电机)
> 记录日期:2026-07-04 · 平台:Jetson (aarch64) · Shell 默认激活 `lingbot-venv`

---

## 0. 硬件与端口对应关系

| 机械臂 | 通信 | 端口 | 说明 |
|---|---|---|---|
| **主臂 StarAI (Violin)** | UART 串口 | `/dev/ttyCH341USB0` | CH340 (`1a86:7523`),用 `ch341` 驱动,节点名**不是** `ttyUSB0` |
| **从臂 星海图 A1Z** | CAN 1Mbps | `can0`(待定) | A1Z 官方文档点名的 USB-CANFD 适配器 VID/PID = `a8fa:8598`,与本机 lsusb 一致 |

检测命令:
```bash
lsusb                       # 看到 CH340 + CANFD Analyser(a8fa:8598)
ls /dev/ttyCH341USB0        # 主臂串口节点
ip -br link show | grep can # can0~can3(Jetson 板载 mttcan),初始全 DOWN
```

---

## 1. 克隆开源驱动仓库

放在 `/home/tommyzihao/ROBOT ARM/`,与 `S1` 并行:

```bash
cd "/home/tommyzihao/ROBOT ARM"
git clone https://github.com/userguide-galaxea/GALAXEA-A1Z.git          # 从臂 CAN SDK
git clone https://github.com/servodevelop/fashionstar-uart-servo-python.git  # 主臂 UART SDK
```

- **GALAXEA-A1Z**:`a1z/motor_drivers/`(CAN 封装 + MotorA/MotorB 驱动)、`tools/motor_diag.py`(诊断)、`tools/set_zero.py`(零点标定)。协议:MIT 位置-速度-力矩混合控制 @1Mbps,250Hz + Pinocchio 重力补偿。
- **fashionstar-uart-servo-python**:核心是 `src/uservo.py`;`example/` 下有 servo_scan / set_servo_angle / query_servo_angle 等示例。

---

## 2. 创建 conda 环境 `GALAXEA-lingbot`

```bash
conda create -y -n GALAXEA-lingbot python=3.10
conda activate GALAXEA-lingbot

# pinocchio 用 conda-forge 装(aarch64 上 pip 装 `pin` 容易失败)
conda install -y -c conda-forge pinocchio

# ⚠️ shell 里默认激活了 lingbot-venv,它的 pip 会盖住 conda 的。
#   在本环境装包一律用 `python -m pip`,不要直接用 pip。
python -m pip install "numpy" "python-can>=4.0" pyserial
python -m pip install -e "/home/tommyzihao/ROBOT ARM/GALAXEA-A1Z" --no-deps   # 不重装 pin,保留 conda 版
```

**已安装版本**:numpy 2.2.6 · python-can 4.6.1 · pyserial 3.5 · pinocchio 4.0.0 · a1z 0.0.1(可编辑)

验证:
```bash
python -c "import numpy, can, serial, pinocchio, a1z; print('all OK')"
```
预期:打印 `all OK`(实测通过 ✅)。

---

## 3. 让 fashionstar 的 `uservo` 全局可导入

示例脚本里用的是相对路径 `sys.path.append("../../src")`,不 cd 进目录就 `ModuleNotFoundError: uservo`。
解决:在环境 site-packages 写一个 `.pth` 指向 src:

```bash
echo "/home/tommyzihao/ROBOT ARM/fashionstar-uart-servo-python/src" \
  > /home/tommyzihao/miniconda3/envs/GALAXEA-lingbot/lib/python3.10/site-packages/uservo_src.pth
python -c "import uservo; print(uservo.__file__)"   # 预期打印 src/uservo.py 路径 ✅
```

另外把示例里写死的 Windows 端口 `COM7` 改成本机节点,已另存改好的脚本:
`/home/tommyzihao/ROBOT ARM/scan_leader_starai.py`(端口 `/dev/ttyCH341USB0`)。

---

## 4. 启用从臂 CAN 接口

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
ip -br link show can0        # 预期状态 UP
```

---

## 5. 测试:扫描电机(上电后,2026-07-04)

### 5.1 主臂 StarAI(UART)—— ✅ 已解决

多波特率实测:
```bash
python "/home/tommyzihao/ROBOT ARM/scan_leader_starai.py"
```
| 波特率 | 扫描结果 |
|---|---|
| **1000000 (1M)** | **舵机 ID `[0,1,2,3,4,5,6]`** —— 7 个(6+1 DoF,对应 Violin leader)✅ |
| 500000 / 250000 / 115200 | `[]` 空 |

**结论:StarAI 官方波特率 = 1,000,000 (1M)。**
- 官方 Seeed wiki 未写明波特率(只给了 `/dev/ttyUSBx`),此值为本机实测确认。
- Violin(leader)与 Viola(follower)舵机型号相同,均 6+1 DoF;舵机 ID 从 0 编到 6。
- `scan_leader_starai.py` 里 `SERVO_BAUDRATE` 已固定为 `1000000`。

### 5.2 从臂 A1Z(CAN)—— ❌ 未解决(卡在硬件通道)

```bash
python "/home/tommyzihao/ROBOT ARM/GALAXEA-A1Z/tools/motor_diag.py" --scan            # can0
# 已额外把 can1/can2/can3 都 up 并逐个扫描
```
- **预期(正常)**:6 个电机响应 —— MotorA ID 0x01~0x03、MotorB ID 0x04~0x06。
- **实测**:`can0/1/2/3` 全部电机 **无响应**;`can0` 末尾报 `No buffer space available [Error Code 105] (ENOBUFS)`。

---

## 6. 从臂 CAN:错误诊断与 gs_usb 驱动解决(核心)

### 6.1 硬件通道确认
从臂 A1Z 的 CAN 走:**电机 → HHS USB-CANFD 适配器(`a8fa:8598`)→ USB 拓展坞 → Jetson USB 口**。
- 即从臂**不在** Jetson 板载 CAN(`can0~can3` = mttcan)上,这也是为什么扫 can0~3 全无响应。
- A1Z README(README.md:91)明确此适配器 = **HHS USB-CANFD**,是 **gs_usb 协议**设备,官方步骤:`modprobe gs_usb` + `echo "a8fa 8598" > /sys/bus/usb/drivers/gs_usb/new_id`(自定义 VID/PID 需手动绑)。

### 6.2 根因:Tegra 内核缺 gs_usb 模块
```bash
lsusb -v -d a8fa:8598   # bInterfaceClass=255 (Vendor Specific), 无内核驱动绑定
sudo modprobe gs_usb    # -> FATAL: Module gs_usb not found (6.8.12-tegra)
zcat /proc/config.gz | grep GS_USB   # -> # CONFIG_CAN_GS_USB is not set
```
设备是 vendor-class,靠 gs_usb 绑定;而本内核没编 gs_usb → 适配器变不成 canX。

### 6.3 解决:out-of-tree 编译 gs_usb.ko(已完成 ✅)
具备条件:`CONFIG_CAN_DEV=m` 已启用、内核 build 目录 + `Module.symvers` 都在、gcc/make 齐全。
```bash
# 1. 取 6.8 的 gs_usb 源码(单文件)
mkdir -p ~/gs_usb_build && cd ~/gs_usb_build
curl -sfL https://raw.githubusercontent.com/torvalds/linux/v6.8/drivers/net/can/usb/gs_usb.c -o gs_usb.c
printf 'obj-m := gs_usb.o\n' > Makefile
# 2. 编译(vermagic 会匹配 6.8.12-tegra)
make -C /lib/modules/$(uname -r)/build M=$PWD modules
# 3. 安装 + 开机自动加载
sudo cp gs_usb.ko /lib/modules/$(uname -r)/kernel/drivers/net/can/usb/ && sudo depmod -a
echo gs_usb | sudo tee /etc/modules-load.d/gs_usb.conf
```
> 编译产物与源码已备份到 `gs_usb_module/`;gs_usb 已装入内核模块树并设为开机自动加载。

### 6.4 一键启用从臂 CAN
```bash
sudo bash "/home/tommyzihao/ROBOT ARM/setup_follower_can.sh"
```
脚本做:`modprobe gs_usb` → 绑 `a8fa 8598` → 找到 USB 那个 CAN 接口 → `up @1Mbps`。
- **实测:适配器生成的接口 = `can4`**(can0~3 是板载 mttcan)。启用后 `can state = ERROR-ACTIVE`(健康)。

### 6.5 ★真正的根因:gs_usb 驱动 USB 端点号写错(已修复✅)

> 前面怀疑物理层是**错的**。真凶是驱动 bug,由嵌入式工程师 agent 定位并修复。

**现象闭环**:TX packets=0 / dropped=15、四波特率抓包全 0、**连 loopback 回环也全 dropped**、错误计数全 0。loopback 都发不出去 → 直接排除总线/接线/终端/电机,问题在**主机侧驱动**。

**根因**:HHS 适配器(`a8fa:8598`)的 bulk OUT 端点是 **EP 0x01**,但 mainline `gs_usb.c` 把发送端点**硬编码成 2**(`GS_USB_ENDPOINT_OUT 2` → EP 0x02,该设备根本没有这个端点)。于是每帧 `usb_submit` 立刻 `-ENOENT(-2)` 失败被计为 tx_dropped,**一个字节都没上线**。
- 证据:`dmesg` 每帧报 `gs_usb can4: usb_submit failed (err=-2)`;`lsusb -v` 显示端点只有 IN 0x81 / **OUT 0x01** / IN 0x82,没有 OUT 0x02。

**修复**(改 `gs_usb_module/gs_usb.c` 后重编安装):
```c
#define GS_USB_ENDPOINT_OUT 1   /* 原为 2 —— 本适配器 bulk OUT 在 EP 0x01 */
```
同时把 `a8fa:8598` 加进驱动 `gs_usb_table[]`,**开机插上自动绑定,不用再手动写 new_id**。
```bash
# 编译要在无空格路径下(make M= 不吃空格);产物已装入内核模块树
sudo cp gs_usb.ko /lib/modules/$(uname -r)/kernel/drivers/net/can/usb/ && sudo depmod -a
```

**验证(2026-07-05)**:`--scan --channel can4` → **6 个电机全部 OK**,读到位置、响应 0.5~13ms、`TX dropped=0`、错误计数全 0。从臂 CAN 链路完全打通。✅

### 6.6(历史)曾怀疑的硬件层排查 —— 已排除
- ✅ 适配器已被内核识别,`can4` 正常(ERROR-ACTIVE,无 bus 错误),驱动/软件链路全通。
- ⚠️ `up` 偶发 `RTNETLINK Connection timed out`(USB 控制传输卡住,疑似接在 USB 拓展坞上)。解决:USB reset 重新枚举即可 —— `sudo python -c "import usb.core; usb.core.find(idVendor=0xa8fa,idProduct=0x8598).reset()"` 然后重写 new_id;**建议把适配器直插 Jetson USB 口,别经拓展坞**。
- ❌ 接好线后扫描仍无电机响应。**决定性证据(2026-07-04)**:
  - 扫描后 `ip -s link show can4`:**TX packets=0 / dropped=15**(发出的帧全部无 ACK),RX=0。
  - 被动抓包 `candump can4`:**1M / 500k / 250k / 125k 四种波特率全部 0 帧** —— 总线在任何速率下彻底静默。
- **判读**:总线零电气活动 = **电机没有真正接入这条 CAN 总线**。非软件、非波特率问题。按概率查物理层:

  1. **CANH/CANL 接反**(最常见,最快验证:把两根线对调再试)。
  2. **120Ω 终端电阻**:CAN 总线两端各需一个;很多 USB-CAN 适配器自带一个终端电阻的 DIP 开关/跳线,确认已打开。
  3. **电机动力电**:确认电机总线的独立动力电已上(不只是 Jetson/逻辑电);电机 CAN 收发器不上电就不会 ACK。
  4. **接头/线序**:适配器端子(CANH/CANL/GND)与 A1Z 机械臂 CAN 接头的针脚定义是否对应,是否插在正确的口。

### 6.6 诊断命令
```bash
ip -s -d link show can4                     # 看 can state / tx-rx 错误计数 / dropped
candump can4                                # 抓总线报文(接对了应能看到电机反馈)
cansend can4 001#00                         # 测发送;TX packets 增长=上线成功,dropped 增长=没人 ACK
python "/home/tommyzihao/ROBOT ARM/GALAXEA-A1Z/tools/motor_diag.py" --scan --channel can4
```
**判读**:接对线后,`candump` 应能看到电机周期反馈、`--scan` 看到 6 个电机(ID 0x01~0x06)、`ip -s link show can4` 的 tx 错误/ dropped 不再增长。

---

## 总结状态

| | 通信 | 端口 | 波特率 | 状态 |
|---|---|---|---|---|
| **主臂 StarAI (Violin)** | UART | `/dev/ttyCH341USB0` | **1M** | ✅ 扫到 7 舵机 (ID 0~6) |
| **从臂 星海图 A1Z** | CAN (gs_usb) | **`can4`** (HHS 适配器) | 1M | ✅ 6 电机全 OK (ID 0x01~0x06),TX dropped=0 |

> 从臂 gs_usb 驱动已修复 USB 端点 bug 并加入 ID 表,插上自动绑定;`sudo ip link set can4 up type can bitrate 1000000` 即可用。

---

## 目录结构

```
/home/tommyzihao/ROBOT ARM/
├── S1                              # 原有
├── GALAXEA-A1Z                     # 从臂 CAN SDK(a1z 包已 pip -e 安装)
├── fashionstar-uart-servo-python   # 主臂 UART SDK(uservo 已 .pth 全局可导入)
├── gs_usb_module/                  # 自编译的 gs_usb 驱动(gs_usb.c / gs_usb.ko / Makefile 备份)
├── scan_leader_starai.py           # 主臂扫描脚本(端口 ttyCH341USB0, 波特率 1M)
├── setup_follower_can.sh           # 从臂 CAN 一键启用脚本(gs_usb + 绑定 + up)
└── SETUP_LOG.md                    # 本文档
```

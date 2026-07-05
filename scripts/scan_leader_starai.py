'''
StarAI 主臂 (leader / Violin) 舵机扫描
- 端口已改为本机 CH340 节点 /dev/ttyCH341USB0
- uservo 已通过 .pth 全局可导入, 无需 cd 到 example 目录
用法:
    conda activate GALAXEA-lingbot
    python "/home/tommyzihao/ROBOT ARM/scan_leader_starai.py"
如扫不到, 依次尝试把 SERVO_BAUDRATE 改为 1000000 / 500000。
'''
import serial
from uservo import UartServoManager

SERVO_PORT_NAME = '/dev/ttyCH341USB0'   # 本机 CH340 串口节点
SERVO_BAUDRATE = 1000000                # StarAI 官方波特率 = 1M (实测扫到 ID 0~6)

uart = serial.Serial(port=SERVO_PORT_NAME, baudrate=SERVO_BAUDRATE,
                     parity=serial.PARITY_NONE, stopbits=1,
                     bytesize=8, timeout=0)
uservo = UartServoManager(uart)

print(f"开始扫描舵机 (port={SERVO_PORT_NAME}, baud={SERVO_BAUDRATE}) ...")
uservo.scan_servo()
servo_list = list(uservo.servos.keys())
print("扫描结束, 舵机 ID 列表: {}".format(servo_list))

# peak_usb 内核模块(PEAK PCAN-USB → SocketCAN)

reBot B601-RS 的 RobStride 电机走 **PCAN-USB → SocketCAN @1Mbps**。很多 Jetson/Tegra 内核默认没编
`CONFIG_CAN_PEAK_USB`(`modprobe peak_usb` 报 `Module not found`),PCAN 插上也不生成 canX。

`install.sh` 会:用当前内核头文件,从 mainline(默认 `v6.8`,与运行内核大版本对应)拉 peak_usb 源码
现编 → 装进模块树 → `depmod` → 设开机自动加载 → `modprobe`。装好后 PCAN-USB 自动生成一个 canX(本机 `can5`)。

```bash
bash install.sh                 # 默认取 v6.8 源码;别的内核用 KVER=v6.6 bash install.sh
sudo ip link set can5 up type can bitrate 1000000 restart-ms 100
```

`peak_usb.ko` 为本机(6.8.12-tegra)预编译产物,仅当 vermagic 匹配才可直接用;否则必须重编(需内核头文件)。

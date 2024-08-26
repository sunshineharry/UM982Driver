# 和芯星通UM982/UM980驱动

- 和芯星通UM982/UM980驱动
- 支持 `NMEA` 和扩展 `NMEA` 指令集: `PVTSLN`, `GNHPR`, `BESTNAV`
- 目前仅支持ASSIC
- 完整获取 UM982 定位信息
    - 位置及标准差（UTM坐标 + WGS84坐标）
    - 速度及标准差
    - 姿态信息（横滚角，俯仰角，偏航角）（需要双天线）

# 多语种文档

- [English](readme.md) ( English version will be coming soom )
- [简体中文](appendix/readme_zh.md)

# 安装

```
pip install um982-driver
```

# 使用方法

## UM982 配置
需要使能UM982的串口输出`PVTSLN`, `GNHPR`, `BESTNAV`语句，指令格式为
```
<输出语句名称> <输出串口> <输出频率>
```
具体指令示例如下（输出到UM982的COM2，波特率921600，0.05秒输出一次）：
```
config com2 921600
PVTSLNA com2 0.05
GPHPR com2 0.05
BESTNAVA com2 0.05
```

## 一个简单的例程

[DEMO](demo/simple_demp/demp.py)

```python
from um982 import UM982Serial
import time
import signal
import sys

def signal_handler(sig, frame):
    um982_driver.stop()                                  # 多线程清理
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)             # 注册退出函数

if __name__ == "__main__":
    um982_driver = UM982Serial("/dev/rtk1", 921600)      # 实例化驱动对象
    um982_driver.start()                                 # 驱动以多线程方式运行


    while True:
        # 获取定位信息：高度(hgt)、纬度(lat)、经度(lon)及其标准差
        bestpos_hgt, bestpos_lat, bestpos_lon, bestpos_hgtstd, bestpos_latstd, bestpos_lonstd = um982_driver.fix
        print(f"Height: {bestpos_hgt} meters, Latitude: {bestpos_lat} degrees, Longitude: {bestpos_lon} degrees")
        print(f"Height Std: {bestpos_hgtstd} meters, Latitude Std: {bestpos_latstd} meters, Longitude Std: {bestpos_lonstd} meters")

        # 获取 UTM 坐标
        utm_x, utm_y = um982_driver.utmpos
        print(f"UTM X: {utm_x} meters, UTM Y: {utm_y} meters")

        # 获取速度信息：东向速度、北向速度、垂直速度及其标准差
        vel_east, vel_north, vel_ver, vel_east_std, vel_north_std, vel_ver_std = um982_driver.vel
        print(f"Velocity East: {vel_east} m/s, Velocity North: {vel_north} m/s, Velocity Vertical: {vel_ver} m/s")
        print(f"Velocity East Std: {vel_east_std} m/s, Velocity North Std: {vel_north_std} m/s, Velocity Vertical Std: {vel_ver_std} m/s")

        # 获取姿态信息：航向角(heading)、俯仰角(pitch)、横滚角(roll)
        heading, pitch, roll = um982_driver.orientation
        print(f"Heading: {heading} degrees, Pitch: {pitch} degrees, Roll: {roll} degrees")
        print('')

        time.sleep(0.2)
```

## ROS2

对于 `ROS2` 下的功能包, 可以查看另一个 [DEMO](demo/ros2/)

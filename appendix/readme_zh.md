# 和芯星通UM982/UM980驱动

- 和芯星通UM982/UM980驱动
- 支持 `NMEA` 和扩展 `NMEA` 指令集: `PVTSLN`, `KSXT`, `GNHPR`, `BESTNAV`
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
需要使能UM982的串口输出`PVTSLN`, `KSXT`, `GNHPR`, `BESTNAV`语句，指令格式为
```
<输出语句名称> <输出串口> <输出频率>
```
具体指令示例如下（输出到UM982的COM2，波特率921600，0.05秒输出一次）：
```
config com2 921600
PVTSLNA com2 0.05
KSXT com2 0.05
GPHPR com2 0.05
BESTNAVA com2 0.05
```

## 一个简单的例程
```python
from um982.assic_driver import UM982Driver
import serial

if __name__ == "__main__":
    um982_driver = UM982Driver()                # 实例化驱动对象
    ser = serial.Serial("/dev/ttyACM0", 921600) # 打开串口
    while True:
        msg = str(ser.read_all(),'utf-8')       # 读取UM982的输出
        um982_driver.decode(msg)                # 解码
        # 输出位置相关的信息
        print(um982_driver.bestpos_lat)         # 维度
        print(um982_driver.utm_x)               # utm坐标的x（东方向为正）
        print(um982_driver.bestpos_latstd)      # 维度的标准差，也可以视为utm坐标下x的标准差
        print(um982_driver.bestpos_lon)         # 经度
        print(um982_driver.utm_y)               # utm坐标的y（北方向为正）
        print(um982_driver.bestpos_lonstd)      # 维度的标准差，也可以视为utm坐标下y的标准差
        print(um982_driver.bestpos_hgt)         # 海拔高度（天方向为正）
        print(um982_driver.bestpos_hgtstd)      # 海拔高度测量的标准差
        # 输出速度相关的信息
        print(um982_driver.vel_east)            # utm坐标下x方向的速度
        print(um982_driver.vel_east_std)        # utm坐标下x方向的速度的标准差
        print(um982_driver.vel_north)           # utm坐标下y方向的速度
        print(um982_driver.vel_north_std)       # utm坐标下y方向的速度的标准差
        print(um982_driver.vel_up)              # 垂直方向的速度
        print(um982_driver.vel_up_std)          # 垂直方向速度的标准差
        # 输出姿态相关的信息
        print(um982_driver.heading)             # 航向角
        print(um982_driver.pitch)               # pitch
        print(um982_driver.roll)                # roll
```

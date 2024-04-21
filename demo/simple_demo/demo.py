import serial


import numpy as np
from pyproj import CRS, Transformer
from um982.assic_driver import UM982Driver

import time



if __name__ == "__main__":
    um982_driver = UM982Driver()                # 实例化驱动对象
    ser = serial.Serial("/dev/rtk", 921600) # 打开串口
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

        time.sleep(0.05)




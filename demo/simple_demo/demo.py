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

import serial


import numpy as np
from pyproj import CRS, Transformer

import time


def check_checksum(nmea_sentence):
    # 移除起始的'$'和'*'及之后的校验和部分
    try:
        sentence, checksum = nmea_sentence[1:].split("*")
    except:
        return False
    calculated_checksum = 0
    # 对字符串中的每个字符进行异或运算
    for char in sentence:
        calculated_checksum ^= ord(char)
    # 将计算得到的校验和转换为十六进制格式，并大写
    calculated_checksum_hex = format(calculated_checksum, 'X')
    # 校验和比较
    return calculated_checksum_hex.zfill(2) == checksum.upper()


def check_crc(nmea_sentence):
    # create crc_table
    def crc_table():
        table = []
        for i in range(256):
            crc = i
            for j in range(8, 0, -1):
                if crc & 1:
                    crc = (crc >> 1) ^ 0xEDB88320
                else:
                    crc >>= 1
            table.append(crc)
        return table

    # Function to calculate CRC using the provided algorithm and the generated table
    def calculate_crc32(data):
        table = crc_table()
        crc = 0
        for byte in data:
            crc = table[(crc ^ byte) & 0xFF] ^ (crc >> 8)
        return crc & 0xFFFFFFFF

    # main check_crc
    try:
        sentence, crc = nmea_sentence[1:].split("*")
    except:
        return False
    return crc == format(calculate_crc32(sentence.encode()), '08x')


def msg_seperate(msg:str):
    return msg[1:msg.find('*')].split(',')


def determine_utm_zone_and_hemisphere(lat, lon):
    """
    Determines the UTM zone and hemisphere (north or south) for a given latitude and longitude.

    Parameters:
    lat (float): Latitude of the point.
    lon (float): Longitude of the point.

    Returns:
    tuple: A tuple containing the UTM zone number and a boolean indicating whether it's in the northern hemisphere.
    """
    # UTM zones are determined by longitude, starting at -180 degrees.
    zone_number = int((lon + 180) / 6) + 1

    # Northern hemisphere is above the equator (latitude 0)
    north = lat >= 0

    return zone_number, north


class UM982Driver:
    def __init__(self) -> None:
        # From PVTSLN message
        self.bestpos_hgt    = 0
        self.bestpos_lat    = 0
        self.bestpos_lon    = 0
        self.bestpos_hgtstd = 0
        self.bestpos_latstd = 0
        self.bestpos_lonstd = 0
        # From KSXT message
        self.vel_east       = 0
        self.vel_north      = 0
        self.vel_up         = 0
        # From GPHPR message
        self.heading        = 0
        self.pitch          = 0
        self.roll           = 0
        # From BESTNAV message
        self.vel_horstd     = 0
        self.vel_verstd     = 0
        # 转换为UTM
        self.utm_x          = 0
        self.utm_y          = 0
        # std
        self.vel_east_std   = 0
        self.vel_north_std  = 0
        self.vel_hor_cov    = 0
        self.vel_up_std     = 0


    def __PVTSLN_solver(self, msg:str):
        parts = msg_seperate(msg)
        self.bestpos_hgt = float(parts[3+7])
        self.bestpos_lat = float(parts[4+7])
        self.bestpos_lon = float(parts[5+7])
        self.bestpos_hgtstd = float(parts[6+7])
        self.bestpos_latstd = float(parts[7+7])
        self.bestpos_lonstd = float(parts[8+7])

    def __KSXT_solver(self, msg:str):
        parts = msg_seperate(msg)
        self.vel_east  = float(parts[18-1])
        self.vel_north = float(parts[19-1])
        self.vel_up    = float(parts[20-1])

    def __GNHPR_solver(self, msg:str):
        parts = msg_seperate(msg)
        self.heading = float(parts[3-1])
        self.pitch   = float(parts[4-1])
        self.roll    = float(parts[5-1])

    def __BESTNAV_solver(self, msg:str):
        parts = msg_seperate(msg)
        self.vel_horstd = float(parts[-1])
        self.vel_verstd = float(parts[-2])

    def __utm_trans(self):
        wgs84_crs              = CRS("epsg:4326")
        zone_number, isnorth   = determine_utm_zone_and_hemisphere(self.bestpos_lat, self.bestpos_lon)
        utm_crs_str            = f"epsg:326{zone_number}" if isnorth else f"epsg:327{zone_number}"
        utm_crs                = CRS(utm_crs_str)
        transformer            = Transformer.from_crs(wgs84_crs, utm_crs, always_xy=True)
        self.utm_x, self.utm_y = transformer.transform(self.bestpos_lon, self.bestpos_lat)

    def __std_trans(self):
        heading_rad = np.deg2rad(self.heading)
        cos_h = np.cos(heading_rad)
        sin_h = np.sin(heading_rad)
        vel_cov_xy         = self.vel_horstd ** 2
        self.vel_east_std  = np.sqrt(vel_cov_xy * cos_h ** 2)       # 东方向速度的方差
        self.vel_hor_cov   = np.sqrt(vel_cov_xy * cos_h * sin_h)    # 速度的协方差
        self.vel_north_std = np.sqrt(vel_cov_xy * sin_h ** 2)       # 北方向速度的方差
        self.vel_up_std    = self.vel_verstd                        # 天方向速度的方差

    def __parse(self, msg: str):
        all_msg = msg
        split_msg = all_msg.strip().split("\r\n")
        for msg in split_msg:
            try:
                if msg.startswith("#PVTSLNA") and check_crc(msg):
                    self.__PVTSLN_solver(msg)
                elif msg.startswith("$GNHPR") and check_checksum(msg):
                    self.__GNHPR_solver(msg)
                elif msg.startswith("$KSXT") and check_checksum(msg):
                    self.__KSXT_solver(msg)
                elif msg.startswith("#BESTNAVA") and check_crc(msg):
                    self.__BESTNAV_solver(msg)
            except:
                print("Warnning: Illegal Sentance.")
            finally:
                pass


    def decode(self, msg:str):
        self.__parse(msg)
        self.__utm_trans()
        self.__std_trans()


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




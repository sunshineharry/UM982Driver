import numpy as np
from pyproj import CRS, Transformer

from .utils import *


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


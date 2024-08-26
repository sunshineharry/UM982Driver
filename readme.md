# Driver for UM982 / UM980

- Driver for UNICORECOMM UM882/UM980 GPS
- Support NMEA, extended NMEA Instruction Set: `PVTSLN`, `GNHPR`, `BESTNAV`
- Support ASSIC Instruction Set Only now
- Get as much location information as possible ( more than stantad NMEA sentence )

# Language

- [English](readme.md) ( English version will be coming soom )
- [简体中文](appendix/readme_zh.md)



# Installation

```
pip install um982-driver
```

# Usage

It is necessary to enable the serial port output of the UM982 for PVTSLN, KSXT, GNHPR, BESTNAV statements, with the command format as

```
<output statement name> <output COM port> <output frequency>
```

Specific command examples are as follows (output to UM982's COM2, baud rate 921600, output once every 0.05 seconds):

```
config com2 921600
PVTSLNA com2 0.05
KSXT com2 0.05
GPHPR com2 0.05
BESTNAVA com2 0.05
```

# Examples
## A simple example


[DEMO](demo/simple_demp/demp.py)

```python
from um982 import UM982Serial
import time
import signal
import sys

def signal_handler(sig, frame):
    um982_driver.stop()                                  # Multi-thread cleaning
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)             # Register the opt-out function

if __name__ == "__main__":
    um982_driver = UM982Serial("/dev/rtk1", 921600)      # Instantiate the driving object
    um982_driver.start()                                 # The driver runs in a multi-threaded manner


    while True:
        # Obtain positioning information: altitude (hgt), latitude (lat), longitude (lon) and their standard deviations
        bestpos_hgt, bestpos_lat, bestpos_lon, bestpos_hgtstd, bestpos_latstd, bestpos_lonstd = um982_driver.fix
        print(f"Height: {bestpos_hgt} meters, Latitude: {bestpos_lat} degrees, Longitude: {bestpos_lon} degrees")
        print(f"Height Std: {bestpos_hgtstd} meters, Latitude Std: {bestpos_latstd} meters, Longitude Std: {bestpos_lonstd} meters")

        # Get UTM coordinates
        utm_x, utm_y = um982_driver.utmpos
        print(f"UTM X: {utm_x} meters, UTM Y: {utm_y} meters")

        # Get velocity information: eastbound velocity, northbound velocity, vertical velocity, and their standard deviations
        vel_east, vel_north, vel_ver, vel_east_std, vel_north_std, vel_ver_std = um982_driver.vel
        print(f"Velocity East: {vel_east} m/s, Velocity North: {vel_north} m/s, Velocity Vertical: {vel_ver} m/s")
        print(f"Velocity East Std: {vel_east_std} m/s, Velocity North Std: {vel_north_std} m/s, Velocity Vertical Std: {vel_ver_std} m/s")

        # Obtain attitude information: heading angle (heading), pitch angle (pitch), roll angle (roll)
        heading, pitch, roll = um982_driver.orientation
        print(f"Heading: {heading} degrees, Pitch: {pitch} degrees, Roll: {roll} degrees")
        print('')

        time.sleep(0.2)
```

## ROS2

For `ROS2` usage, you can find another [DEMO](demo/ros2/)




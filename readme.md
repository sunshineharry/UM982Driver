# Driver for UM982 / UM980

- Driver for UNICORECOMM UM882/UM980 GPS
- Support NMEA, extended NMEA Instruction Set: `PVTSLN`, `KSXT`, `GNHPR`, `BESTNAV`
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

## UM982 Configuration
It is necessary to enable the serial port output of the UM982 for `PVTSLN`, `KSXT`, `GNHPR`, `BESTNAV` statements, with the command format as
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

## Examples
[demo.py](demo/simple_demo/demo.py)
```
from um982.assic_driver import UM982Driver
import serial

if __name__ == "__main__":
    um982_driver = UM982Driver()                # Instantiate the driver object
    ser = serial.Serial("/dev/ttyACM0", 921600) # Open the serial port
    while True:
        msg = str(ser.read_all(),'utf-8')       # Read UM982's output
        um982_driver.decode(msg)                # Decode
        # Output position-related information
        print(um982_driver.bestpos_lat)         # Latitude
        print(um982_driver.utm_x)               # UTM coordinate x (positive eastward)
        print(um982_driver.bestpos_latstd)      # Standard deviation of latitude, can also be seen as the standard deviation of UTM coordinate x
        print(um982_driver.bestpos_lon)         # Longitude
        print(um982_driver.utm_y)               # UTM coordinate y (positive northward)
        print(um982_driver.bestpos_lonstd)      # Standard deviation of longitude, can also be seen as the standard deviation of UTM coordinate y
        print(um982_driver.bestpos_hgt)         # Elevation (positive upward)
        print(um982_driver.bestpos_hgtstd)      # Standard deviation of elevation measurement
        # Output speed-related information
        print(um982_driver.vel_east)            # Speed in the UTM x direction
        print(um982_driver.vel_east_std)        # Standard deviation of speed in the UTM x direction
        print(um982_driver.vel_north)           # Speed in the UTM y direction
        print(um982_driver.vel_north_std)       # Standard deviation of speed in the UTM y direction
        print(um982_driver.vel_up)              # Vertical speed
        print(um982_driver.vel_up_std)          # Standard deviation of vertical speed
        # Output attitude-related information
        print(um982_driver.heading)             # Heading
        print(um982_driver.pitch)               # Pitch
        print(um982_driver.roll)                # Roll
```
For `ROS2` usage, you can find another [DEMO](demo/ros2/).
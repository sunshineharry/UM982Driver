import sys
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler, euler_from_quaternion


from um982 import UM982Serial


class UM982DriverROS2(Node):

    def _ros_log_debug(self, log_data):
        self.get_logger().debug(str(log_data))

    def _ros_log_info(self, log_data):
        self.get_logger().info(str(log_data))

    def _ros_log_warn(self, log_data):
        self.get_logger().warn(str(log_data))

    def _ros_log_error(self, log_data):
        self.get_logger().error(str(log_data))


    def __init__(self) -> None:
        super().__init__('um982_serial_driver')
        # Step1：从参数服务器获取port和baud
        self.declare_parameter('port', '/dev/rtk1')
        self.declare_parameter('baud', 961200)
        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value
        # Step2：打开串口
        try:
            self.um982serial = UM982Serial(port, baud)
            self._ros_log_info(f'serial {port} open successfully!')
        except:
            self._ros_log_error(f'serial {port} do not open!')
            sys.exit(0)
        # Step3：新建一个线程用于处理串口数据
        self.um982serial.start()
        # Step4：ROS相关
        self.fix_pub        = self.create_publisher(NavSatFix, '/gps/fix',     10)
        self.utm_pub        = self.create_publisher(Odometry,  '/gps/utmpos',  10)
        self.pub_timer      = self.create_timer(1/20, self.pub_task)

    def pub_task(self):
        bestpos_hgt, bestpos_lat, bestpos_lon, bestpos_hgtstd, bestpos_latstd, bestpos_lonstd = self.um982serial.fix
        utm_x, utm_y = self.um982serial.utmpos
        vel_east, vel_north, vel_ver, vel_east_std, vel_north_std, vel_ver_std = self.um982serial.vel
        heading, pitch, roll = self.um982serial.orientation
        this_time = self.get_clock().now().to_msg()

        # Step 1: Publish GPS Fix Data
        fix_msg = NavSatFix()
        fix_msg.header.stamp = this_time
        fix_msg.header.frame_id = 'gps'
        fix_msg.latitude = bestpos_lat
        fix_msg.longitude = bestpos_lon
        fix_msg.altitude = bestpos_hgt
        fix_msg.position_covariance[0] = float(bestpos_latstd)**2
        fix_msg.position_covariance[4] = float(bestpos_lonstd)**2
        fix_msg.position_covariance[8] = float(bestpos_hgtstd)**2
        fix_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self.fix_pub.publish(fix_msg)

        # Step 2: Publish UTM Position Data
        odom_msg = Odometry()
        odom_msg.header.stamp = this_time
        odom_msg.header.frame_id = 'earth'
        odom_msg.child_frame_id  = 'base_link'
        odom_msg.pose.pose.position.x = utm_x
        odom_msg.pose.pose.position.y = utm_y
        odom_msg.pose.pose.position.z = bestpos_hgt
        quaternion = quaternion_from_euler(math.radians(roll), math.radians(pitch), math.radians(heading))
        odom_msg.pose.pose.orientation.x = quaternion[0]
        odom_msg.pose.pose.orientation.y = quaternion[1]
        odom_msg.pose.pose.orientation.z = quaternion[2]
        odom_msg.pose.pose.orientation.w = quaternion[3]
        odom_msg.pose.covariance         = [0.0] * 36
        odom_msg.pose.covariance[0]      = float(bestpos_latstd)**2
        odom_msg.pose.covariance[7]      = float(bestpos_lonstd)**2
        odom_msg.pose.covariance[14]     = float(bestpos_hgtstd)**2
        odom_msg.pose.covariance[21]     = 0.1
        odom_msg.pose.covariance[28]     = 0.1
        odom_msg.pose.covariance[35]     = 0.1
        odom_msg.twist.twist.linear.x    = vel_east
        odom_msg.twist.twist.linear.y    = vel_north
        odom_msg.twist.twist.linear.z    = vel_ver
        odom_msg.twist.covariance        = [0.0] * 36
        odom_msg.twist.covariance[0]     = float(vel_east_std)**2
        odom_msg.twist.covariance[7]     = float(vel_north_std)**2
        odom_msg.twist.covariance[14]    = float(vel_ver_std)**2
        self.utm_pub.publish(odom_msg)

        # Test
        _, _, yaw = euler_from_quaternion(quaternion)
        print(heading)

    def run(self):
        if rclpy.ok():
            rclpy.spin(self)

    def stop(self):
        self.um982serial.stop()
        self.pub_timer.cancel()



import time
import signal

def signal_handler(sig, frame):
    um982_driver.stop()
    time.sleep(0.1)
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)
rclpy.init()
um982_driver = UM982DriverROS2()


def main():
    um982_driver.run()

if __name__ == "__main__":
    main()

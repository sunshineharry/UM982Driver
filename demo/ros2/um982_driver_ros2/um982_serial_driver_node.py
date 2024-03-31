import threading
from beeprint import pp as print
import time
import serial


import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from tf_transformations import quaternion_from_euler
from rclpy.qos import qos_profile_system_default

from um982.assic_driver import UM982Driver



class UM982SerialDriverNode(threading.Thread, Node):
    def __init__(self, ser:serial.Serial) -> None:
        threading.Thread.__init__(self)
        Node.__init__(self, 'um982_serial_driver_node')
        self.ser       = ser
        self.solver    = UM982Driver()
        self.isrunning = True
        self.odom_pub  = self.create_publisher(Odometry, 'odom', qos_profile_system_default)


    def get_nmea_msg(self) -> bool:
        all_msg = str(self.ser.read_all(),'utf-8')
        if ( len(all_msg) == 0 ):
            return False
        else:
            self.solver.parse(all_msg)
            self.solver.utm_trans()
            self.solver.std_trans()
            return True


    def nmea_msg_to_odom(self) -> None:
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'utm'
        msg.child_frame_id  = 'base_link'

        # 设置位置
        msg.pose.pose.position.x = self.solver.utm_x
        msg.pose.pose.position.y = self.solver.utm_y
        msg.pose.pose.position.z = self.solver.bestpos_hgt

        # 使用roll, pitch, heading设置方向
        q = quaternion_from_euler(self.solver.roll, self.solver.pitch, self.solver.heading)
        msg.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        # 填充位置的方差
        msg.pose.covariance[0]  = self.solver.bestpos_latstd ** 2    # position.x的方差
        msg.pose.covariance[7]  = self.solver.bestpos_lonstd ** 2    # position.y的方差
        msg.pose.covariance[14] = self.solver.bestpos_hgtstd ** 2   # position.z的方差

        small_var = 1e-6  # 一个非常小的方差值
        msg.pose.covariance[21] = small_var  # roll的方差
        msg.pose.covariance[28] = small_var  # pitch的方差
        msg.pose.covariance[35] = small_var  # yaw的方差

        # 填充速度
        msg.twist.twist.linear.x = self.solver.vel_north
        msg.twist.twist.linear.y = self.solver.vel_east
        msg.twist.twist.linear.z = self.solver.vel_up

        # 填充速度标准差
        msg.twist.covariance[0]  = self.solver.vel_east_std  ** 2   # linear.x 的方差
        msg.twist.covariance[1]  = self.solver.vel_hor_cov ** 2     # linear.x和linear.y的协方差
        msg.twist.covariance[6]  = self.solver.vel_hor_cov ** 2     # linear.y和linear.x的协方差
        msg.twist.covariance[7]  = self.solver.vel_north_std ** 2   # linear.y 的方差
        msg.twist.covariance[14] = self.solver.vel_up_std ** 2      # linear.z 的方差

        self.odom_pub.publish(msg)


    def run(self):
        while self.isrunning:
            if( self.get_nmea_msg() ):
                self.nmea_msg_to_odom()
            time.sleep(0.02)

    def terminate(self):
        self.isrunning = False
        self.ser.close()



def main():
    rclpy.init()
    parm_getter = Node("parm_getter_for_um982_serial_driver")
    serial_port = parm_getter.declare_parameter('serial_port', '/dev/ttyACM0').get_parameter_value().string_value
    baud_rate   = parm_getter.declare_parameter('baud_rate', 921600).get_parameter_value().integer_value
    parm_getter.destroy_node()
    um982_serial_driver = UM982SerialDriverNode(serial.Serial(serial_port, baud_rate))
    try:
        um982_serial_driver.start()
        rclpy.spin(um982_serial_driver)
    except KeyboardInterrupt:
        pass
    finally:
        um982_serial_driver.terminate()
        um982_serial_driver.join()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

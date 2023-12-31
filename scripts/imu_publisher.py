#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from geometry_msgs.msg import (Quaternion, Vector3)

import board
import busio
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_ROTATION_VECTOR,
)
from adafruit_bno08x.i2c import BNO08X_I2C

class IMUPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')
        self.publisher_ = self.create_publisher(Imu, 'imu', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.bno = BNO08X_I2C(busio.I2C(board.SCL, board.SDA, frequency = 400000))
        self.bno.enable_feature(BNO_REPORT_ACCELEROMETER)
        self.bno.enable_feature(BNO_REPORT_GYROSCOPE)
        self.bno.enable_feature(BNO_REPORT_MAGNETOMETER)
        self.bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)

    def timer_callback(self):
        msg = Imu()

        tempQ = Quaternion()
        tempQ.x = self.bno.quaternion[0]
        tempQ.y = self.bno.quaternion[1]
        tempQ.z = self.bno.quaternion[2]
        tempQ.w = self.bno.quaternion[3]
        msg.orientation = tempQ

        tempAngVel = Vector3()
        tempAngVel.x = self.bno.gyro[0]
        tempAngVel.y = self.bno.gyro[1]
        tempAngVel.z = self.bno.gyro[2]
        msg.angular_velocity = tempAngVel

        tempAccel = Vector3()
        tempAccel.x = self.bno.acceleration[0]
        tempAccel.y = self.bno.acceleration[1]
        tempAccel.z = self.bno.acceleration[2]
        msg.linear_acceleration = tempAccel

        self.publisher_.publish(msg)
    

def main(args=None):
    rclpy.init(args=args)

    imu_publisher = IMUPublisher()

    rclpy.spin(imu_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    imu_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
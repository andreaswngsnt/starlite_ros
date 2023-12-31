#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from geometry_msgs.msg import (Quaternion, Vector3)


class IMUSubscriber(Node):

    def __init__(self):
        super().__init__('imu_subscriber')
        self.subscription = self.create_subscription(
            Imu,
            'imu',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info("Acceleration: (%0.6f, %0.6f, %0.6f) m/s^2" % (msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z))
        self.get_logger().info("Angular Velocity: (%0.6f, %0.6f, %0.6f) m/s^2" % (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z))
        self.get_logger().info("Quaternion: (%0.6f, %0.6f, %0.6f, %0.6f) m/s^2" % (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w))


def main(args=None):
    rclpy.init(args=args)

    imu_subscriber = IMUSubscriber()

    rclpy.spin(imu_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    imu_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
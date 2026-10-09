#!/usr/bin/env python3
"""

Input  : /input_ik  (geometry_msgs/Twist)
         linear.x  = kecepatan maju robot v (m/s)
         angular.z = kecepatan putar robot w (rad/s)

Output : /left_wheel/command   (std_msgs/Float64) kecepatan roda kiri  (rad/s)
         /right_wheel/command  (std_msgs/Float64) kecepatan roda kanan (rad/s)

Rumus IK:
    w_kiri  = (v - w * L / 2) / r
    w_kanan = (v + w * L / 2) / r

Rumus FK (untuk verifikasi):
    v = r * (w_kanan + w_kiri) / 2
    w = r * (w_kanan - w_kiri) / L

dengan r = jari-jari roda, L = jarak antar roda.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


class InverseKinematics(Node):
    def __init__(self):
        super().__init__('inverse_kinematics')

        # Parameter fisik robot (bisa diubah lewat --ros-args -p nama:=nilai)
        self.declare_parameter('wheel_radius', 0.03)       # meter
        self.declare_parameter('wheel_separation', 0.17)   # meter
        self.declare_parameter('max_wheel_speed', 20.0)    # rad/s

        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.max_wheel_speed = self.get_parameter('max_wheel_speed').value

        # Subscriber: perintah kecepatan badan robot
        self.create_subscription(
            Twist,
            '/input_ik',
            self.velocity_callback,
            10
        )

        # Publisher: perintah kecepatan tiap roda
        self.left_publisher = self.create_publisher(
            Float64,
            '/left_wheel/command',
            10
        )
        self.right_publisher = self.create_publisher(
            Float64,
            '/right_wheel/command',
            10
        )

        self.get_logger().info(
            f'Inverse kinematics aktif. '
            f'r={self.wheel_radius} m, L={self.wheel_separation} m, '
            f'batas roda={self.max_wheel_speed} rad/s'
        )

    def velocity_callback(self, msg):
        v = msg.linear.x
        w = msg.angular.z

        # Inverse kinematics: kecepatan badan -> kecepatan roda
        v_left = (v - (w * self.wheel_separation / 2.0)) / self.wheel_radius
        v_right = (v + (w * self.wheel_separation / 2.0)) / self.wheel_radius

        # Pembatas kecepatan: kecilkan kedua roda dengan rasio yang sama
        # supaya arah gerak (lurus/melengkung) tidak berubah
        peak = max(abs(v_left), abs(v_right))
        limited = peak > self.max_wheel_speed
        if limited:
            scale = self.max_wheel_speed / peak
            v_left *= scale
            v_right *= scale

        # Forward kinematics: verifikasi hasil IK
        v_check = self.wheel_radius * (v_right + v_left) / 2.0
        w_check = self.wheel_radius * (v_right - v_left) / self.wheel_separation

        # Publish ke masing-masing roda
        left_msg = Float64()
        left_msg.data = float(v_left)
        self.left_publisher.publish(left_msg)

        right_msg = Float64()
        right_msg.data = float(v_right)
        self.right_publisher.publish(right_msg)

        self.get_logger().info(
            f'v: {v:.2f}, w: {w:.2f} -> '
            f'Left: {v_left:.2f} rad/s, Right: {v_right:.2f} rad/s | '
            f'cek FK: v={v_check:.2f}, w={w_check:.2f}'
            + (' [DIBATASI]' if limited else '')
        )


def main(args=None):
    rclpy.init(args=args)
    node = InverseKinematics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

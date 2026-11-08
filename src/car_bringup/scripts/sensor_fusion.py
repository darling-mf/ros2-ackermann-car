#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""传感器融合 v2：轮子管里程、陀螺管方向（实车标准架构）。

  航向：陀螺积分为主（快、无刮地误差），轮式航向慢速锚定防零偏漂移
  位置：轮式速度沿【融合航向】积分（而不是沿轮式航向！）

设计依据：
  1. 持续转弯时轮式里程计航向率存在系统性高估（前轮刮地，物理决定，
     实车同样存在）→ 位置沿轮式航向积分会把航向误差放大成大位置误差；
  2. 互补滤波若强锚定轮式航向，会忠实复现其系统偏差；航向锚定强度
     必须弱（alpha 默认 0.999），零偏靠上车静置标定而不是靠强锚定；
  3. dt 必须用消息头（sim time）；墙钟在 RTF<1 时会让积分发散。

实机迁移：STM32 上传校准后的 ω_gyro + 轮速，本节点原样跑在树莓派；
进阶则换 robot_localization EKF（原理相同、协方差更规范）。
"""

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


def wrap(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def yaw_to_quat(yaw):
    q = Odometry().pose.pose.orientation
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class SensorFusion(Node):
    def __init__(self):
        super().__init__('sensor_fusion')
        self.declare_parameter('alpha', 0.999)
        self.declare_parameter('wheel_odom_topic', '/wheel/odom')
        self.declare_parameter('publish_tf', True)
        self.alpha = self.get_parameter('alpha').get_parameter_value().double_value
        wheel_topic = self.get_parameter('wheel_odom_topic').get_parameter_value().string_value
        self.publish_tf = self.get_parameter('publish_tf').get_parameter_value().bool_value

        self.odom = None
        self.wz = 0.0
        self.last_imu_stamp = None     # sim time（消息头）
        self.yaw_f = None
        self.pos_f = None
        self.yaw_g = None              # 纯陀螺对照

        self.create_subscription(Odometry, wheel_topic, self._odom, 20)
        self.create_subscription(Imu, '/imu', self._imu, 20)
        self.pub_fused = self.create_publisher(Odometry, '/odometry/fused', 10)
        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)
        self.pub_gyro = self.create_publisher(Odometry, '/odometry/gyro_only', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

    def _odom(self, m):
        self.odom = m

    def _imu(self, m):
        stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        self.wz = m.angular_velocity.z
        if self.odom is None:
            return
        odom_yaw = yaw_of(self.odom.pose.pose.orientation)
        if self.yaw_f is None:
            # 初始化：位置/航向起点取自轮式里程计
            self.yaw_f = odom_yaw
            self.yaw_g = odom_yaw
            self.pos_f = (self.odom.pose.pose.position.x,
                          self.odom.pose.pose.position.y)
            self.last_imu_stamp = stamp
            return

        dt = stamp - self.last_imu_stamp
        self.last_imu_stamp = stamp
        if not (0.0 < dt < 0.5):       # 乱序/长断流保护
            return

        # ---- 航向：陀螺积分为主 + 轮式弱锚定 ----
        self.yaw_f = wrap(
            self.yaw_f
            + self.alpha * self.wz * dt
            + (1.0 - self.alpha) * wrap(odom_yaw - self.yaw_f))
        self.yaw_g = wrap(self.yaw_g + self.wz * dt)

        # ---- 位置：轮速沿融合航向积分 ----
        v = self.odom.twist.twist.linear.x
        x = self.pos_f[0] + v * math.cos(self.yaw_f) * dt
        y = self.pos_f[1] + v * math.sin(self.yaw_f) * dt
        self.pos_f = (x, y)

        fused = Odometry()
        fused.header.stamp = m.header.stamp
        fused.header.frame_id = 'odom'
        fused.child_frame_id = 'base_footprint'
        fused.pose.pose.position.x = x
        fused.pose.pose.position.y = y
        fused.pose.pose.orientation = yaw_to_quat(self.yaw_f)
        fused.twist = self.odom.twist
        self.pub_fused.publish(fused)
        self.pub_odom.publish(fused)

        if self.publish_tf:
            tf = TransformStamped()
            tf.header.stamp = m.header.stamp
            tf.header.frame_id = 'odom'
            tf.child_frame_id = 'base_footprint'
            tf.transform.translation.x = x
            tf.transform.translation.y = y
            tf.transform.translation.z = 0.0
            tf.transform.rotation = fused.pose.pose.orientation
            self.tf_broadcaster.sendTransform(tf)

        gyro = Odometry()
        gyro.header = fused.header
        gyro.child_frame_id = 'base_footprint'
        gyro.pose = self.odom.pose
        gyro.pose.pose.orientation = yaw_to_quat(self.yaw_g)
        self.pub_gyro.publish(gyro)


def main():
    rclpy.init()
    node = SensorFusion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

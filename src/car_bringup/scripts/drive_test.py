#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""阿克曼控制链路自检脚本。

依次做三件事：
  1. 向 /cmd_vel 持续发布指定 Twist；
  2. 同时读取 /odom 和 /joint_states；
  3. 打印前后位姿、前轮转角、后轮速度，并检查方向是否符合预期。

用法示例：
  ros2 run car_bringup drive_test.py --vx 0.10 --wz 0.0  --duration 2.0   # 直行
  ros2 run car_bringup drive_test.py --vx 0.10 --wz 0.3  --duration 2.0   # 左转
  ros2 run car_bringup drive_test.py --vx -0.10 --wz 0.0 --duration 1.5   # 倒车
"""

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def wrap_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class DriveTest(Node):
    def __init__(self, args):
        super().__init__('ackermann_drive_test')
        self.args = args
        self.odom = None
        self.joints = None

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.create_subscription(JointState, '/joint_states', self._on_joints, 10)

    def _on_odom(self, msg):
        self.odom = msg

    def _on_joints(self, msg):
        self.joints = msg

    def wait_for_odom(self, timeout=5.0):
        deadline = time.monotonic() + timeout
        while self.odom is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.odom is not None

    def _pose(self):
        p = self.odom.pose.pose.position
        q = self.odom.pose.pose.orientation
        return p.x, p.y, yaw_from_quaternion(q)

    def _joint_map(self):
        if self.joints is None:
            return {}
        return dict(zip(self.joints.name, zip(self.joints.position, self.joints.velocity)))

    def print_state(self, tag):
        x, y, yaw = self._pose()
        tw = self.odom.twist.twist
        print(
            f'{tag}: odom x={x:.4f} y={y:.4f} yaw={yaw:.4f} '
            f'vx={tw.linear.x:.4f} wz={tw.angular.z:.4f}'
        )
        jm = self._joint_map()
        for name in (
            'right_front_steer_joint',
            'left_front_steer_joint',
            'right_rear_wheel_joint',
            'left_rear_wheel_joint',
        ):
            if name in jm:
                pos, vel = jm[name]
                print(f'   {name}: pos={pos: .4f} vel={vel: .4f}')

    def spin_for(self, seconds, twist=None, rate=20.0, sample=True):
        period = 1.0 / rate
        deadline = time.monotonic() + seconds
        next_sample = 0.4
        start = time.monotonic()
        while True:
            now = time.monotonic()
            if now >= deadline:
                break
            if twist is not None:
                self.pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=min(period, max(0.0, deadline - now)))
            elapsed = time.monotonic() - start
            if sample and elapsed >= next_sample:
                self.print_state(f'  t={elapsed:4.2f}s')
                next_sample += 0.4

    def run(self):
        a = self.args
        if not self.wait_for_odom():
            print('错误：5 秒内没有收到 /odom。')
            print('请先确认 controller 已激活：ros2 control list_controllers')
            return 1

        self.print_state('起始')
        x0, y0, yaw0 = self._pose()

        msg = Twist()
        msg.linear.x = a.vx
        msg.angular.z = a.wz
        self.spin_for(a.duration, msg, a.rate)

        # 发布零速度，确保测试结束后停车
        stop = Twist()
        self.spin_for(0.5, stop, a.rate, sample=False)
        self.print_state('结束')

        x1, y1, yaw1 = self._pose()
        dx, dy = x1 - x0, y1 - y0
        distance = math.hypot(dx, dy)
        along = dx * math.cos(yaw0) + dy * math.sin(yaw0)
        dyaw = wrap_angle(yaw1 - yaw0)

        print('\n===== 结果判断 =====')
        print(f'位移向量 ({dx:+.4f}, {dy:+.4f})，路程投影 {along:+.4f} m，转角 {dyaw:+.4f} rad')
        if abs(a.vx) > 1e-3:
            if a.vx * along < 0:
                print('❌ 前进方向与指令相反：检查后轮关节 axis 符号。')
            else:
                print('✅ 前后方向正确。')
        if abs(a.wz) > 1e-3:
            if a.wz * dyaw < 0:
                print('❌ 转向与指令相反：检查前轮 steer_joint 的 axis / 左右轮顺序。')
            else:
                print('✅ 转向方向正确。')

        if abs(a.vx) > 1e-3:
            phi = math.atan(a.wz * a.wheelbase / a.vx)
            print(f'理论虚拟中心转角 atan(wz*L/vx) = {phi:+.4f} rad （前轮限位 ±{a.max_steer:.2f}）')
            print('正常现象：wz=0 时前轮应回到 0；纯旋转(vx=0)时阿克曼车无法原地转，'
                  '前轮只会打到限位而车轮不转。')
        print(f'实际移动距离 {distance:.4f} m')
        return 0


def main():
    parser = argparse.ArgumentParser(description='阿克曼小车 /cmd_vel -> /odom 闭环自检')
    parser.add_argument('--vx', type=float, default=0.10, help='线速度 m/s')
    parser.add_argument('--wz', type=float, default=0.0, help='角速度 rad/s')
    parser.add_argument('--duration', type=float, default=2.0, help='持续时间 s')
    parser.add_argument('--rate', type=float, default=20.0, help='cmd_vel 发布频率 Hz')
    parser.add_argument('--wheelbase', type=float, default=0.112, help='轴距 m，用于打印理论转角')
    parser.add_argument('--max-steer', type=float, default=0.5, help='前轮限位 rad')
    args = parser.parse_args()

    rclpy.init()
    node = DriveTest(args)
    try:
        return node.run()
    except KeyboardInterrupt:
        return 130
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())

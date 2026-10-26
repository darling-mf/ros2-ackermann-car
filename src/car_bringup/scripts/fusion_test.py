#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""传感器融合量化实验：圆周运动下对比三个航向源对真值的误差。

场景设计：v=0.3, wz=0.6 → 半径 0.5m 的圆。连续转弯会放大轮式里程计的
刮地误差，陀螺零偏会随时间线性暴露，真值由 p3d 提供——三个航向源
（轮式/纯陀螺/融合）在同一段时间里各自的弱点都会显形。

用法（先启动仿真 + sensor_fusion.py）：
  ros2 run car_bringup fusion_test.py --loops 3
"""

import argparse
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


def wrap(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class FusionTest(Node):
    def __init__(self):
        super().__init__('fusion_test')
        self.gt = None        # (t, x, y, yaw) 真值
        self.odom_yaw = None
        self.fused_yaw = None
        self.gyro_yaw = None
        self.odom_xy = None
        self.fused_xy = None
        self.samples = []     # dict per truth sample
        self.gt_last = 0.0
        self.create_subscription(Odometry, '/ground_truth', self._gt, 20)
        self.create_subscription(Odometry, '/odom', self._odom, 20)
        self.create_subscription(Odometry, '/odometry/fused', self._fused, 20)
        self.create_subscription(Odometry, '/odometry/gyro_only',
                                 lambda m: self.__dict__.update(gyro_yaw=yaw_of(m.pose.pose.orientation)), 20)

    @staticmethod
    def _store(m, tag):
        obj = tag
        obj['yaw'] = yaw_of(m.pose.pose.orientation)
        obj['x'] = m.pose.pose.position.x
        obj['y'] = m.pose.pose.position.y
        return obj

    def _odom(self, m):
        self.odom_yaw, self.odom_xy = yaw_of(m.pose.pose.orientation), \
            (m.pose.pose.position.x, m.pose.pose.position.y)

    def _fused(self, m):
        self.fused_yaw, self.fused_xy = yaw_of(m.pose.pose.orientation), \
            (m.pose.pose.position.x, m.pose.pose.position.y)

    def _gt(self, m):
        t = time.monotonic()
        if t - self.gt_last < 0.1:      # 50Hz 太密，10Hz 采样足够
            return
        self.gt_last = t
        if None in (self.odom_yaw, self.fused_yaw, self.gyro_yaw):
            return
        self.samples.append({
            'yaw_gt': yaw_of(m.pose.pose.orientation),
            'x_gt': m.pose.pose.position.x,
            'y_gt': m.pose.pose.position.y,
            'yaw_odom': self.odom_yaw,
            'yaw_fused': self.fused_yaw,
            'yaw_gyro': self.gyro_yaw,
            'xy_odom': self.odom_xy,
            'xy_fused': self.fused_xy,
        })


def rmse(errs):
    return math.sqrt(sum(e * e for e in errs) / max(len(errs), 1)) if errs else float('nan')


def main():
    p = argparse.ArgumentParser(description='融合量化实验（圆周）')
    p.add_argument('--loops', type=float, default=3.0, help='圈数')
    p.add_argument('--v', type=float, default=0.3)
    p.add_argument('--wz', type=float, default=0.6)
    args = p.parse_args()

    duration = args.loops * 2.0 * math.pi / args.wz + 2.0
    rclpy.init()
    node = FusionTest()
    pub = node.create_publisher(Twist, '/cmd_vel', 10)

    def spin(sec):
        end = time.monotonic() + sec
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.01)

    spin(1.5)   # 等真值/里程计/融合节点就绪
    node.samples.clear()
    print(f'圆周实验: v={args.v} wz={args.wz}（R={args.v/args.wz:.2f}m）'
          f' {args.loops} 圈 ≈ {duration:.1f}s')
    t0 = time.monotonic()
    msg = Twist()
    msg.linear.x = args.v
    msg.angular.z = args.wz
    while time.monotonic() - t0 < duration:
        pub.publish(msg)
        spin(0.02)
    msg.linear.x = 0.0
    msg.angular.z = 0.0
    for _ in range(40):
        pub.publish(msg)
        spin(0.02)

    if len(node.samples) < 20:
        print('错误：样本不足，确认 /ground_truth（p3d）和 sensor_fusion.py 都在运行')
        return 1

    def rmse(errs):
        return math.sqrt(sum(e * e for e in errs) / len(errs))

    # ===== 坐标系对齐（关键！）=====
    # teleport 复位的是仿真位姿，odom 系不跟着复位——每次实验开始时
    # odom 系与世界系差一个固定旋转+平移。直接比较"odom 姿态 vs 真值"
    # 会把坐标系偏移误判成漂移。正确做法：按 t0 时刻把每个源刚体对齐到
    # 真值系，之后只报【对齐后的漂移】。
    s0 = node.samples[0]
    yaw_off = {k: wrap(s0[k] - s0['yaw_gt'])
               for k in ('yaw_odom', 'yaw_gyro', 'yaw_fused')}
    dpsi0 = wrap(s0['yaw_gt'] - s0['yaw_odom'])          # odom→world 旋转
    p0 = (s0['x_gt'], s0['y_gt'])
    c, sn = math.cos(dpsi0), math.sin(dpsi0)

    def to_world(p_odom):
        dx = p_odom[0] - s0['xy_odom'][0]
        dy = p_odom[1] - s0['xy_odom'][1]
        return (p0[0] + c * dx - sn * dy, p0[1] + sn * dx + c * dy)

    print(f'\n（初始坐标系偏移: odom yaw = {math.degrees(yaw_off["yaw_odom"]):+.1f}°，'
          f'已对齐，以下为对齐后漂移）')
    print(f'===== 航向漂移 vs 真值（{len(node.samples)} 个采样, {args.loops} 圈）=====')
    rows = [
        ('轮式里程计 /odom',        'yaw_odom'),
        ('纯陀螺 /odometry/gyro_only', 'yaw_gyro'),
        ('融合 v2 /odometry/fused',  'yaw_fused'),
    ]
    for name, key in rows:
        e = [wrap((s[key] - s['yaw_gt']) - yaw_off[key]) for s in node.samples]
        print(f'{name:30s} RMSE={math.degrees(rmse(e)):6.2f}° | '
              f'最大={math.degrees(max(abs(x) for x in e)):6.2f}° | '
              f'末端={math.degrees(e[-1]):+6.2f}°')

    print('===== 位置漂移 vs 真值（对齐后）=====')
    for name, key in [('轮式里程计 /odom', 'xy_odom'),
                      ('融合 v2 /odometry/fused', 'xy_fused')]:
        dists = []
        for s in node.samples:
            w = to_world(s[key])
            dists.append(math.hypot(w[0] - s['x_gt'], w[1] - s['y_gt']))
        print(f'{name:30s} RMSE={rmse(dists):6.4f} m | '
              f'最大={max(dists):6.4f} m | 末端={dists[-1]:6.4f} m')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

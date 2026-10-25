#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""R5 控制器验收测试：三项客观判据 + 一份 PASS/FAIL 报告。

测试 1【加速度限幅】发 vx 阶跃，从 /odom 采样实际速度曲线，
     验证峰值加速度不超过 r5.max_accel（允许滤波平滑带来的余量）。
测试 2【速度相关转向限幅】保持 vx 前进，突然给一个“几何上需要更大转角”
     的 wz，验证稳态 ω 被压到理论上限 ω_max = v·tan(φ_lim)/L 附近。
测试 3【转向速率限幅】在测试 2 的转角阶跃里统计 dφ/dt 峰值，
     验证不超过 r5.max_steer_rate。

用法（先启动 gazebo.launch.py，两个控制器 active 后）：
  ros2 run car_bringup acceptance_test.py --vx 0.5 --wz-big 3.0 \
      --max-accel 0.8 --max-steer-rate 9.5
"""

import argparse
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState


def slope(series):
    """series: [(t, value)] -> 峰值变化率与均值斜率列表"""
    rates = []
    for (t0, v0), (t1, v1) in zip(series, series[1:]):
        dt = t1 - t0
        if dt > 1e-4:
            rates.append((v1 - v0) / dt)
    return rates


class AcceptanceTest(Node):
    def __init__(self, a):
        super().__init__('r5_acceptance_test')
        self.a = a
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom = []      # [(t, vx, wz)]
        self.steer = []     # [(t, phi_right, phi_left)]
        self.create_subscription(Odometry, '/odom', self._odom_cb, 20)
        self.create_subscription(JointState, '/joint_states', self._js_cb, 20)

    def _odom_cb(self, m):
        self.odom.append(
            (time.monotonic(), m.twist.twist.linear.x, m.twist.twist.angular.z))

    def _js_cb(self, m):
        d = dict(zip(m.name, m.position))
        pr = d.get('right_front_steer_joint')
        pl = d.get('left_front_steer_joint')
        if pr is not None and pl is not None:
            self.steer.append((time.monotonic(), pr, pl))

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def publish(self, vx, wz):
        m = Twist()
        m.linear.x = vx
        m.angular.z = wz
        self.pub.publish(m)

    def wait_odom(self, timeout=5.0):
        end = time.monotonic() + timeout
        while not self.odom and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        return bool(self.odom)


def main():
    p = argparse.ArgumentParser(description='R5 控制器三项验收')
    p.add_argument('--vx', type=float, default=0.5, help='测试 1/2 的目标速度 m/s')
    p.add_argument('--wz-big', type=float, default=3.0, help='测试 2 的角速度阶跃 rad/s')
    p.add_argument('--max-accel', type=float, default=0.8, help='控制器里的 r5.max_accel')
    p.add_argument('--max-steer-rate', type=float, default=9.5, help='控制器里的 r5.max_steer_rate')
    p.add_argument('--a-lat-max', type=float, default=2.0, help='控制器里的 r5.a_lat_max')
    p.add_argument('--max-steer-mech', type=float, default=0.235, help='单前轮机械限位 rad')
    p.add_argument('--wheelbase', type=float, default=0.112, help='轴距 m')
    args = p.parse_args()

    rclpy.init()
    node = AcceptanceTest(args)
    results = []

    try:
        if not node.wait_odom():
            print('错误：5 秒内没收到 /odom，先确认两个控制器 active')
            return 1
        node.odom.clear()

        # ============ 测试 1：加速度限幅 ============
        print(f'\n===== 测试 1：加速度限幅（vx 阶跃 0 -> {args.vx}）=====')
        t0 = time.monotonic()
        while time.monotonic() - t0 < 2.5:
            node.publish(args.vx, 0.0)
            node.spin_for(0.02)
        seg = [(t, vx) for t, vx, _ in node.odom]
        rates = slope(seg)
        peak_acc = max(rates) if rates else float('nan')
        final_v = seg[-1][1] if seg else float('nan')
        ok1 = peak_acc <= args.max_accel * 1.25 + 0.05 and final_v >= args.vx * 0.85
        results.append(('加速度限幅', f'峰值 {peak_acc:.3f} <= 限值 {args.max_accel:.2f}',
                        f'稳态 vx {final_v:.3f}', ok1))

        # ============ 测试 2：速度相关转向限幅 ============
        print(f'===== 测试 2：转向限幅（vx={args.vx} 保持，wz 阶跃 0 -> {args.wz_big}）=====')
        v_ref = final_v if final_v > 0.1 else args.vx
        phi_lim = min(args.max_steer_mech,
                      math.atan(args.a_lat_max * args.wheelbase / max(v_ref**2, 1e-4)))
        w_max_theory = v_ref * math.tan(phi_lim) / args.wheelbase
        print(f'  理论: v={v_ref:.2f} 时 φ_lim={phi_lim:.3f} rad, '
              f'ω_max={w_max_theory:.3f} rad/s')
        t0 = time.monotonic()
        while time.monotonic() - t0 < 3.0:
            node.publish(args.vx, args.wz_big)
            node.spin_for(0.02)
        tail = [(t, wz) for t, _, wz in node.odom if t > time.monotonic() - 0.6]
        steady_wz = sum(w for _, w in tail) / max(len(tail), 1)
        limited = args.wz_big > w_max_theory
        ok2 = steady_wz <= w_max_theory * 1.15 + 0.02 and (
            (not limited) or steady_wz >= w_max_theory * 0.75)
        results.append(('速度相关转向限幅',
                        f'稳态 ω {steady_wz:.3f} vs 上限 {w_max_theory:.3f}',
                        f'目标 {args.wz_big} {"超限→应被压回" if limited else "未超限"}', ok2))

        # ============ 测试 3：转向速率限幅 ============
        print('===== 测试 3：转向速率限幅（统计测试 2 阶跃期 dφ/dt）=====')
        rates_r = slope([(t, p) for t, p, _ in node.steer])
        rates_l = slope([(t, p) for t, _, p in node.steer])
        peak_rate = max(abs(r) for r in (rates_r + rates_l)) if (rates_r + rates_l) \
            else float('nan')
        ok3 = peak_rate <= args.max_steer_rate * 1.2 + 0.1
        results.append(('转向速率限幅', f'峰值 |dφ/dt| {peak_rate:.3f}',
                        f'<= 限值 {args.max_steer_rate:.2f}', ok3))

        # ============ 停车 ============
        node.odom.clear()
        t0 = time.monotonic()
        while time.monotonic() - t0 < 1.5:
            node.publish(0.0, 0.0)
            node.spin_for(0.02)

        # ============ 报告 ============
        print('\n========== 验收报告 ==========')
        all_ok = True
        for name, got, expect, ok in results:
            all_ok &= ok
            mark = 'PASS' if ok else 'FAIL'
            print(f'[{mark}] {name}: {got}（预期 {expect}）')
        print('全部通过' if all_ok else '存在失败项，检查上方数值与控制器参数')
        return 0 if all_ok else 2
    finally:
        node.publish(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())

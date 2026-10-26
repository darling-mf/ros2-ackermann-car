#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""检查 /plan 里是否包含倒车段（Nav2 倒车问题的分流工具）。

nav2 的路径约定：倒车段的 pose 朝向与该段的行进方向（切线）相差约 180°，
MPPI / RPP 就是靠这个约定识别"这段路要倒着走"。

用法：
    ros2 run car_bringup plan_inspector.py
每收到一条全局路径打印一次分析：
    F = 前进段（pose 朝向 ≈ 路径切线方向）
    R = 倒车段（pose 朝向与切线夹角 > 90°）

判读：
    全 F、换向 0 次  → 规划器根本没给倒车段（查 Smac 的 reverse_penalty）
    有 R 段          → 规划器给了，问题转移去查 MPPI 是否肯执行（看 /cmd_vel）
"""

import math

import rclpy
from nav_msgs.msg import Path
from rclpy.node import Node


def wrap_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def yaw_of(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class PlanInspector(Node):
    def __init__(self):
        super().__init__('plan_inspector')
        self.declare_parameter('topic', '/plan')
        topic = self.get_parameter('topic').get_parameter_value().string_value
        self.create_subscription(Path, topic, self.on_plan, 10)
        self.get_logger().info(f'listening on {topic}')

    def on_plan(self, msg):
        poses = msg.poses
        if len(poses) < 2:
            print('plan: 少于 2 个点，忽略')
            return

        # 合并连续同向的段：[mode, 起点弧长, 终点弧长]
        segments = []
        s = 0.0
        for i in range(len(poses) - 1):
            p0, p1 = poses[i].pose, poses[i + 1].pose
            dx = p1.position.x - p0.position.x
            dy = p1.position.y - p0.position.y
            seg = math.hypot(dx, dy)
            if seg < 1e-4:
                continue
            heading = math.atan2(dy, dx)
            reverse = abs(wrap_angle(yaw_of(p0.orientation) - heading)) > math.pi / 2
            mode = 'R' if reverse else 'F'
            if segments and segments[-1][0] == mode:
                segments[-1][2] = s + seg
            else:
                segments.append([mode, s, s + seg])
            s += seg

        if not segments:
            print('plan: 无有效位移，忽略')
            return

        total = s
        fwd = sum(b - a for m, a, b in segments if m == 'F')
        rev = total - fwd
        desc = ' '.join(f'{m}[{a:.2f}-{b:.2f}]' for m, a, b in segments[:10])
        more = ' ...' if len(segments) > 10 else ''
        print(f'plan 总长 {total:.2f} m | 前进 {fwd:.2f} m | 倒车 {rev:.2f} m | '
              f'换向 {len(segments) - 1} 次')
        print(f'  分段: {desc}{more}')


def main():
    rclpy.init()
    node = PlanInspector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""把 /map (OccupancyGrid) 转成 Marker POINTS，供 RViz 显示建图结果。

为什么需要它：在 VMware/Mesa 下，RViz 的 Map 显示会报
  indexed_8bit_image.frag GLSL link: active samplers with a different type
  refer to the same texture image unit
这是 Ogre 1.12 的已知 shader 问题，地图数据本身没问题，但栅格纹理显示会花。
本节点把地图转成普通 Marker 点（不受该 shader 影响）：
  - 空闲格：中灰
  - 占用格：红色
  - 未知格：不画（透明）
RViz 里对应 /map_points 的 Marker 显示，见 rviz/mapping.rviz。
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA


class MapToPoints(Node):
    def __init__(self):
        super().__init__('map_to_points')
        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = ReliabilityPolicy.RELIABLE
        self.sub = self.create_subscription(OccupancyGrid, '/map', self.cb, qos)
        self.pub = self.create_publisher(Marker, '/map_points', 10)
        self.count = 0

    def cb(self, msg):
        info = msg.info
        pts, cols = [], []
        for j in range(info.height):
            row = j * info.width
            for i in range(info.width):
                v = msg.data[row + i]
                if v < 0:
                    continue
                pts.append(Point(
                    x=info.origin.position.x + (i + 0.5) * info.resolution,
                    y=info.origin.position.y + (j + 0.5) * info.resolution,
                    z=0.01))
                if v >= 65:
                    cols.append(ColorRGBA(r=1.0, g=0.1, b=0.1, a=1.0))
                else:
                    cols.append(ColorRGBA(r=0.55, g=0.55, b=0.55, a=1.0))

        m = Marker()
        m.header.frame_id = msg.header.frame_id or 'map'
        # 用地图消息的时间戳，避免与 RViz 的 sim time 不同步被消息过滤器丢掉
        stamp = msg.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            stamp = self.get_clock().now().to_msg()
        m.header.stamp = stamp
        m.ns = 'map'
        m.id = 0
        m.type = Marker.POINTS
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        # 0.10 m 比 0.05 m 的栅格略大，相邻点会重叠，视觉上接近实心栅格
        m.scale.x = m.scale.y = m.scale.z = 0.10
        m.points = pts
        m.colors = cols
        m.color = ColorRGBA(r=0.55, g=0.55, b=0.55, a=1.0)
        m.lifetime.sec = 0
        self.pub.publish(m)
        self.count += 1
        self.get_logger().info(
            f'#{self.count}: {len(pts)} points, map {info.width}x{info.height}')


def main():
    rclpy.init()
    node = MapToPoints()
    rclpy.spin(node)


if __name__ == '__main__':
    main()

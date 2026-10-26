#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS 2 串口桥：ROS 话题 ↔ OpenCTR B20（X-Protocol / UART 230400 8N1）。

职责（也是 S5 的完成判据）：
  1. /cmd_vel (Twist) → 50Hz 发 0x50 速度帧（vx,vy=0,vw，m/s*1000，大端）；
  2. 收 0x10 综合帧 → 发布三个话题：
       /r5/wheel_vel  (TwistStamped)   轮式里程计速度（固件 FK）
       /r5/imu_raw    (Imu)            陀螺 rad/s + 加速度 m/s^2
       /r5/battery    (BatteryState)   电压 V
  3. 看门狗：cmd_vel 断流超过 watchdog 秒 → 自动发零速帧
     【这是本桥最重要的安全功能：固件没有看门狗，断流后车会保持
     最后速度一直跑（ax_uart_pi.c 收到即写 RobotV_TG，无超时逻辑）——
     上位机必须在桥里兜底。S6 固件改造时再下沉到下位机。】
  4. 链路质量统计：好帧/坏帧/重同步，5s 打印一次。

虚拟/真实零差别切换：
  --port /dev/pts/3        ← virtual_stm32 打印的 pty 路径（联调用）
  --port /dev/ttyUSB0      ← 真实 CH340（实车）
"""

import math
import struct
import threading
import time

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import BatteryState, Imu

from r5_serial.protocol import (
    FrameParser, decode_utx_data, pack_velocity, pack_imu_calibrate,
    ID_UTX_DATA)
from r5_serial.serial_port import SerialPort

CMD_RATE = 50.0          # 0x50 发送频率（固件任务 50Hz，对齐）


class SerialBridge(Node):
    def __init__(self):
        super().__init__('r5_serial_bridge')
        self.declare_parameter('port', '')
        self.declare_parameter('baud', 230400)
        self.declare_parameter('watchdog', 0.5)
        self.declare_parameter('send_imu', True)
        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value
        self.watchdog = self.get_parameter('watchdog') \
            .get_parameter_value().double_value
        if not port:
            self.get_logger().error(
                '必须指定 --port（pty 路径或 /dev/ttyUSB0）')
            raise SystemExit(1)

        self.ser = SerialPort(port, baud)
        self.ser.drain()
        self.parser = FrameParser()
        self.last_cmd_time = 0.0
        self.last_cmd = (0.0, 0.0)
        self.watchdog_kicks = 0
        self.frames_ok = 0
        self.last_stat = time.monotonic()

        qos = QoSProfile(depth=10)
        self.pub_wheel = self.create_publisher(
            TwistStamped, '/r5/wheel_vel', qos)
        self.pub_imu = self.create_publisher(Imu, '/r5/imu_raw', qos)
        self.pub_bat = self.create_publisher(BatteryState, '/r5/battery', qos)
        self.create_subscription(Twist, '/cmd_vel', self._cmd_cb, qos)

        # 读线程：阻塞在串口上，解析出帧后交给 ROS 执行器发布
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

        # 50Hz 指令发送定时器
        self.create_timer(1.0 / CMD_RATE, self._tx_tick)
        self.get_logger().info(
            f'串口桥已启动: port={port} baud={baud} watchdog={self.watchdog}s')

    # ---------- /cmd_vel 订阅 ----------
    def _cmd_cb(self, msg):
        self.last_cmd = (msg.linear.x, msg.angular.z)
        self.last_cmd_time = time.monotonic()

    # ---------- 50Hz 发送（含看门狗） ----------
    def _tx_tick(self):
        now = time.monotonic()
        vx, wz = self.last_cmd
        if self.last_cmd_time == 0.0 or \
                (now - self.last_cmd_time) > self.watchdog:
            if self.watchdog_kicks == 0 or (vx, wz) != (0.0, 0.0):
                self.get_logger().warning(
                    f'看门狗触发：cmd_vel 断流 > {self.watchdog}s，下发零速',
                    throttle_duration_sec=5.0)
            vx, wz = 0.0, 0.0
            self.watchdog_kicks += 1
        try:
            self.ser.write(pack_velocity(vx, 0.0, wz))
        except OSError as e:
            self.get_logger().error(f'串口写失败: {e}', throttle_duration_sec=5.0)

    # ---------- 接收线程 ----------
    def _rx_loop(self):
        while True:
            try:
                data = self.ser.read(timeout=0.05)
            except OSError as e:
                self.get_logger().error(f'串口读失败: {e}')
                time.sleep(1.0)
                continue
            if not data:
                continue
            for fid, payload in self.parser.feed(data):
                if fid == ID_UTX_DATA:
                    self._on_utx(payload)
            now = time.monotonic()
            if now - self.last_stat > 5.0:
                s = self.parser.stats
                self.get_logger().info(
                    f'链路统计: 好帧 {s["frames"]} | 坏帧 {s["bad_checksum"]} | '
                    f'重同步 {s["resync"]} | 字节 {s["bytes"]} | '
                    f'看门狗 {self.watchdog_kicks} 次')
                self.last_stat = now

    def _on_utx(self, payload):
        try:
            d = decode_utx_data(payload)
        except ValueError as e:
            self.get_logger().warning(f'0x10 解码失败: {e}')
            return
        self.frames_ok += 1
        stamp = self.get_clock().now().to_msg()

        tw = TwistStamped()
        tw.header.stamp = stamp
        tw.header.frame_id = 'base_footprint'
        tw.twist.linear.x = d['vel'][0]
        tw.twist.angular.z = d['vel'][2]
        self.pub_wheel.publish(tw)

        if self.get_parameter('send_imu').get_parameter_value().bool_value:
            imu = Imu()
            imu.header.stamp = stamp
            imu.header.frame_id = 'imu_link'
            imu.angular_velocity.x, imu.angular_velocity.y, \
                imu.angular_velocity.z = d['gyro']
            imu.linear_acceleration.x, imu.linear_acceleration.y, \
                imu.linear_acceleration.z = d['acc']
            # 固件未输出姿态四元数（MPU6050 裸数据），协方差置 -1 表示未知
            imu.orientation_covariance[0] = -1.0
            imu.angular_velocity_covariance[0] = 1e-4
            imu.linear_acceleration_covariance[0] = 1e-2
            self.pub_imu.publish(imu)

        bat = BatteryState()
        bat.header.stamp = stamp
        bat.voltage = d['volt']
        bat.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        self.pub_bat.publish(bat)


def main():
    rclpy.init()
    node = SerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 退出前给下位机发零速（安全习惯）
        try:
            node.ser.write(pack_velocity(0.0, 0.0, 0.0))
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

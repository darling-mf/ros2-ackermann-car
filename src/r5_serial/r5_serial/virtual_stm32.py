#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""虚拟 STM32：在 pty 虚拟串口上扮演 OpenCTR B20 固件。

目的：在拿到真硬件之前，把"上位机整条链路"（X-Protocol 编解码、
串口读写、看门狗、ROS 话题）先联调到可用。虚拟固件行为对照真实源码：

  - 50Hz 发 0x10 综合帧（20 字节：acc/gyro/vel/volt，大端 int16）
  - 解析 0x50 速度帧（vx,vy,vw，m/s*1000），一阶惯性跟踪目标速度
  - 支持 0x51（IMU 校准）/ 0x5f（舵机零偏）指令并打印
  - 阿克曼差速正运动学：vel_x = (vA+vB)/2, vel_w = (vB-vA)/W
  - 可注入坏帧（测试上位机解析器的鲁棒性）
  - 记录"零速指令"作为上位机看门狗生效的证据

用法：
  ros2 run r5_serial virtual_stm32                 # 打印 pty 路径后阻塞运行
  ros2 run r5_serial virtual_stm32 --corrupt 20    # 千分之 20 概率发坏帧
"""

import argparse
import math
import os
import pty
import random
import select
import struct
import threading
import time

from r5_serial.protocol import (
    FrameParser, pack_frame, ID_UTX_DATA, UTX_FIELDS,
    GYRO_SCALE_DPS, ACC_SCALE_G, R_VX_LIMIT, R_VW_LIMIT)

WHEEL_TRACK = 0.160   # 固件 ax_robot.h AKM_WHEEL_BASE
TAU = 0.15            # 速度一阶惯性时间常数（模拟电机+PID 响应）


class VirtualSTM32:
    def __init__(self, args):
        self.args = args
        self.target_vx = 0.0
        self.target_wz = 0.0
        self.vel_x = 0.0
        self.vel_w = 0.0
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.yaw = 0.0
        self.gyro_bias = args.bias_gz       # raw 单位
        self.parser = FrameParser()
        self.last_nonzero_cmd = 0.0
        self.watchdog_events = 0
        self.rx_frames = 0
        self.tx_frames = 0
        self.lock = threading.Lock()

        # pty：master 由模拟器持有读写，slave 路径给 serial_bridge 打开
        self.master, self.slave = pty.openpty()
        self.slave_name = os.ttyname(self.slave)
        print(f'[virtual_stm32] 虚拟串口已创建: {self.slave_name}', flush=True)
        print('[virtual_stm32] 用它作为 serial_bridge 的 --port 参数', flush=True)

    def handle_frame(self, frame_id, payload):
        self.rx_frames += 1
        if frame_id == 0x50 and len(payload) >= 6:
            vx, vy, vw = struct.Struct('>hhh').unpack(payload[:6])
            with self.lock:
                now = time.monotonic()
                if (vx or vw) and now - self.last_nonzero_cmd > 0.3:
                    print(f'[virtual_stm32] [WATCHDOG 证据] 收到非零指令 '
                          f'vx={vx} vw={vw}（距上次非零 '
                          f'{now - self.last_nonzero_cmd:.2f}s）')
                if vx == 0 and vw == 0 and now - self.last_nonzero_cmd > 0.3:
                    self.watchdog_events += 1
                    print(f'[virtual_stm32] >>> 上位机看门狗触发：收到零速指令 '
                          f'（第 {self.watchdog_events} 次）<<<')
                if vx or vw:
                    self.last_nonzero_cmd = now
                self.target_vx = max(-R_VX_LIMIT, min(R_VX_LIMIT, vx)) / 1000.0
                self.target_wz = max(-R_VW_LIMIT, min(R_VW_LIMIT, vw)) / 1000.0
        elif frame_id == 0x51:
            print(f'[virtual_stm32] 收到 IMU 校准指令 flag={payload[0] if payload else 0}'
                  '（模拟重新零偏采样 0.5s）')
        elif frame_id == 0x5F:
            if len(payload) >= 2:
                off = struct.Struct('>h').unpack(payload[:2])[0]
                print(f'[virtual_stm32] 收到舵机零偏写值 {off} (0.1°)')
        else:
            print(f'[virtual_stm32] 收到未知帧 id=0x{frame_id:02x} len={len(payload)}')

    def rx_once(self, timeout=0.005):
        r, _, _ = select.select([self.master], [], [], timeout)
        if not r:
            return
        try:
            data = os.read(self.master, 512)
        except OSError:
            return
        for fid, payload in self.parser.feed(data):
            self.handle_frame(fid, payload)

    def build_utx(self, dt):
        # 一阶惯性逼近目标速度（模拟电机 PID 闭环的响应）
        k = dt / TAU
        self.vel_x += (self.target_vx - self.vel_x) * min(k, 1.0)
        self.vel_w += (self.target_wz - self.vel_w) * min(k, 1.0)
        # 简化位姿积分
        self.yaw += self.vel_w * dt
        self.pos_x += self.vel_x * math.cos(self.yaw) * dt
        self.pos_y += self.vel_x * math.sin(self.yaw) * dt

        # 传感器仿真：raw = 物理量 / 满量程 * 32768 + 零偏 + 噪声
        n = self.args.noise
        gyro_z_raw = int((self.vel_w / (GYRO_SCALE_DPS * math.pi / 180.0))
                         + self.gyro_bias + random.gauss(0, n))
        accx_raw = int(random.gauss(0, n * 2))
        accy_raw = int(random.gauss(0, n * 2))
        accz_raw = int(16384 + random.gauss(0, n * 2))   # 静止时 1g ≈ 16384
        velw_raw = int(round(self.vel_w * 1000))
        velx_raw = int(round(self.vel_x * 1000))
        volt_raw = int(1242 + random.gauss(0, 1))        # 12.42V，慢放电略去
        payload = UTX_FIELDS.pack(
            accx_raw, accy_raw, accz_raw,
            0, 0, gyro_z_raw,
            velx_raw, 0, velw_raw, volt_raw)

        frame = pack_frame(ID_UTX_DATA, payload)
        # 注入坏帧：翻转载荷中一个字节，测试上位机解析器
        if self.args.corrupt and random.random() * 1000.0 < self.args.corrupt:
            frame = bytearray(frame)
            i = random.randrange(4, len(frame) - 1)
            frame[i] ^= 0xFF
            frame = bytes(frame)
        return frame

    def tx_once(self):
        now = time.monotonic()
        dt = min(max(now - self._last_t, 0.001), 0.1)
        self._last_t = now
        frame = self.build_utx(dt)
        try:
            os.write(self.master, frame)
            self.tx_frames += 1
        except BlockingIOError:
            pass    # 缓冲满（对端未读），丢本帧——真实 UART 也会溢出
        if now - self._last_stat > 5.0:
            self._last_stat = now
            print(f'[virtual_stm32] 统计: 发帧 {self.tx_frames} 收帧 '
                  f'{self.rx_frames} 看门狗触发 {self.watchdog_events} 次 | '
                  f'目标 vx={self.target_vx:.3f} wz={self.target_wz:.3f} | '
                  f'位姿 ({self.pos_x:.2f},{self.pos_y:.2f},{self.yaw:.2f})')

    def run(self):
        self._last_t = time.monotonic()
        self._last_stat = self._last_t
        period = 1.0 / self.args.rate
        while True:
            loop_start = time.monotonic()
            self.rx_once(0.002)
            self.tx_once()
            # 50Hz 节拍（墙钟即可：模拟器时间基准独立于 ROS）
            elapsed = time.monotonic() - loop_start
            if elapsed < period:
                time.sleep(period - elapsed)


def main():
    p = argparse.ArgumentParser(description='虚拟 STM32（OpenCTR B20 仿真器）')
    p.add_argument('--baud', type=int, default=230400)
    p.add_argument('--rate', type=float, default=50.0, help='0x10 帧频率 Hz')
    p.add_argument('--noise', type=float, default=30.0, help='传感器 raw 噪声 σ')
    p.add_argument('--bias-gz', type=float, default=55.0,
                   help='陀螺 z 零偏 raw（55 ≈ 0.84°/s，模拟未校准 MPU6050）')
    p.add_argument('--corrupt', type=float, default=0.0,
                   help='每千帧注入坏帧的概率（0~1000）')
    args = p.parse_args()

    sim = VirtualSTM32(args)
    try:
        sim.run()
    except KeyboardInterrupt:
        print('\n[virtual_stm32] 退出')


if __name__ == '__main__':
    main()

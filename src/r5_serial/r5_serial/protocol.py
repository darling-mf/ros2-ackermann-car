#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""X-Protocol 编解码层（对照厂商固件源码逐行核对）。

协议事实全部提取自 OpenCTR_B20V11_AKM V1.00.241015 固件源码：
  - Driver/ax_uart_pi.c  AX_UART_PI_SendPacket / USART2_IRQHandler
  - Robot/ax_robot.h     帧号定义（ID_UTX_DATA=0x10 等）
  - Robot/robot.h        ID_ROS2CRP_VEL=0x50 / IMU=0x51 / AKM=0x5f

帧结构（变帧长）：
    AA 55 | LEN | ID | DATA... | CK
    LEN   = 数据长度 + 5（帧头2 + LEN1 + ID1 + CK1）→ 总帧字节数
    CK    = sum(AA,55,LEN,ID,DATA...) & 0xFF
    DATA  = 大端（高字节在前）

固件发送侧（SendPacket）：
    tx_buf[2] = len + 5；校验和对前 4+len 字节累加。
固件接收侧（中断状态机）：
    AA -> 55 -> LEN -> (LEN-1-3) 字节数据（ID 在 [3]）-> 校验末字节；
    校验和初值 = (0xAA+0x55) + LEN，随后逐字节累加。
"""

import math
import struct

HEADER = b'\xAA\x55'

# 帧号（与固件 ax_robot.h / robot.h 完全一致）
ID_UTX_DATA = 0x10      # STM32 -> ROS：50Hz 综合数据（IMU+速度+电压）
ID_URX_VEL = 0x50       # ROS -> STM32：目标速度 vx,vy,vw（m/s*1000）
ID_URX_IMU_CAL = 0x51   # ROS -> STM32：IMU 零偏校准指令（手册未写，源码发现）
ID_URX_AKM_OFFSET = 0x5F  # ROS -> STM32：舵机零偏写值（手册未写，源码发现）

# 0x10 帧载荷布局（20 字节，10 个大端 int16）
UTX_DATA_LEN = 20
UTX_FIELDS = struct.Struct('>hhhhhhhhhh')

# 固件侧常量（Robot/ax_robot.h）
R_VX_LIMIT = 1500       # m/s*1000
R_VY_LIMIT = 1200       # m/s*1000（仅全向底盘，阿克曼固件忽略）
R_VW_LIMIT = 6280       # rad/s*1000
GYRO_SCALE_DPS = 500.0 / 32768.0    # raw -> deg/s
ACC_SCALE_G = 2.0 / 32768.0         # raw -> g
VOLT_SCALE = 0.01                   # raw -> V

BAUD_PI = 230400        # UART4(USART2, EXP2) 与 UART1(调试USB) 同为 230400


def pack_frame(frame_id, payload):
    """按固件 SendPacket 规则打包。payload 为原始字节（不含帧头/长度/校验）。"""
    length = len(payload) + 5
    if length > 60:
        raise ValueError('帧过长（固件接收缓冲 60 字节上限）')
    body = HEADER + bytes([length, frame_id]) + payload
    return body + bytes([sum(body) & 0xFF])


def pack_velocity(vx, vy, vw):
    """打包 0x50 速度帧。vx/vy/vw 单位 m/s 与 rad/s，内部 ×1000 截断为 int16。

    阿克曼固件只使用 vx 与 vw（vy 被忽略，但为协议完整性仍发送 0）。
    限幅对齐固件 R_VX_LIMIT / R_VW_LIMIT。
    """
    def clamp16(v, lim):
        v = int(round(v * 1000.0))
        return max(-lim, min(lim, v))

    payload = struct.Struct('>hhh').pack(
        clamp16(vx, R_VX_LIMIT), clamp16(vy, R_VY_LIMIT),
        clamp16(vw, R_VW_LIMIT))
    return pack_frame(ID_URX_VEL, payload)


def pack_imu_calibrate(flag=1):
    """0x51：请求下位机做 IMU 零偏校准（等价于重新上电校准）。"""
    return pack_frame(ID_URX_IMU_CAL, bytes([flag & 0xFF]))


def pack_akm_offset(offset_deg_x10):
    """0x5f：写舵机零偏（0.1° 单位，对应固件 ax_servo_offset）。"""
    payload = struct.Struct('>h').pack(int(offset_deg_x10))
    return pack_frame(ID_URX_AKM_OFFSET, payload)


class FrameParser:
    """逐字节状态机，逻辑与固件 USART2_IRQHandler 对称。

    用法：feed(串口读到的字节) -> 返回本次解析出的 (frame_id, payload) 列表。
    统计字段用于链路质量监控：frames / bad_checksum / resync / bytes。
    """

    MAX_LEN = 60

    def __init__(self):
        self._state = 0            # 0 等 AA / 1 等 55 / 2 等 LEN / 3 收体
        self._buf = bytearray()
        self._ck = 0
        self.stats = {'frames': 0, 'bad_checksum': 0, 'resync': 0, 'bytes': 0}

    def feed(self, data):
        out = []
        for b in data:
            self.stats['bytes'] += 1
            if self._state == 0:
                if b == 0xAA:
                    self._state = 1
                else:
                    self.stats['resync'] += 1
            elif self._state == 1:
                if b == 0x55:
                    self._state = 2
                else:
                    self._state = 0
                    self.stats['resync'] += 1
            elif self._state == 2:
                if 5 <= b <= self.MAX_LEN:
                    self._buf = bytearray(HEADER)
                    self._buf.append(b)
                    self._ck = 0xAA + 0x55 + b
                    self._state = 3
                else:
                    self._state = 0
                    self.stats['resync'] += 1
            else:
                # 已收 LEN 后的字节；最后一个字节是校验
                if len(self._buf) < self._buf[2] - 1:
                    self._buf.append(b)
                    self._ck = (self._ck + b) & 0xFF
                else:
                    self._state = 0
                    if b == self._ck:
                        self.stats['frames'] += 1
                        # _buf 只含到"最后一个数据字节"（校验和未入缓冲），
                        # 载荷 = 去掉 AA 55 LEN ID 后的全部 → buf[4:]
                        out.append((self._buf[3], bytes(self._buf[4:])))
                    else:
                        self.stats['bad_checksum'] += 1
        return out


def decode_utx_data(payload):
    """解码 0x10 帧 20 字节载荷 -> dict（物理单位）。

    固件换算（手册 4.3 节 + ax_robot.c）：
      acc  = raw / 32768 * 2   (g)   —— 这里再乘 9.81 转 m/s^2
      gyro = raw / 32768 * 500 (°/s) —— 已做上电零偏校准；转 rad/s 输出
      vel  = raw / 1000        (m/s, rad/s)
      volt = raw / 100         (V)
    注意：固件已完成 IMU 轴重映射（ROS.X=IMU.Y, ROS.Y=-IMU.X），
    收到的 X/Y/Z 即 ROS 右手系，无需再转。
    """
    if len(payload) != UTX_DATA_LEN:
        raise ValueError(f'0x10 载荷长度 {len(payload)} != {UTX_DATA_LEN}')
    f = UTX_FIELDS.unpack(payload)
    g = 9.81
    return {
        'acc':   tuple(v * ACC_SCALE_G * g for v in f[0:3]),      # m/s^2
        'gyro':  tuple(v * GYRO_SCALE_DPS * math.pi / 180.0
                       for v in f[3:6]),                          # rad/s
        'vel':   tuple(v / 1000.0 for v in f[6:9]),               # m/s, rad/s
        'volt':  f[9] * VOLT_SCALE,                               # V
    }


if __name__ == '__main__':
    # 自检 1：复现固件文档中的示例帧 AA 55 0B 01 03 E8 FC 18 00 0A 14
    example = pack_frame(0x01, struct.Struct('>hhh').pack(1000, -1000, 10))
    expect = bytes.fromhex('AA550B0103E8FC18000A14')
    assert example == expect, f'{example.hex()} != {expect.hex()}'
    print('协议自检 1 通过：与固件手册示例帧逐字节一致')

    # 自检 2：pack → parse 回环（0x10 完整 20 字节载荷）
    payload = bytes(range(20))
    p = FrameParser()
    frames = p.feed(pack_frame(ID_UTX_DATA, payload))
    assert frames == [(ID_UTX_DATA, payload)], frames
    assert p.stats['frames'] == 1 and p.stats['bad_checksum'] == 0
    print('协议自检 2 通过：0x10 二十字节载荷回环一致')

    # 自检 3：坏帧必须被拒绝且不影响后续好帧
    good = pack_frame(ID_URX_VEL, struct.Struct('>hhh').pack(200, 0, 400))
    bad = bytearray(good)
    bad[7] ^= 0xFF                      # 翻转载荷一个字节 → 校验失败
    p2 = FrameParser()
    frames2 = p2.feed(bytes(bad) + good)   # 坏帧紧跟好帧（含字节流重同步）
    assert p2.stats['bad_checksum'] == 1 and p2.stats['frames'] == 1
    assert frames2 == [(ID_URX_VEL, struct.Struct('>hhh').pack(200, 0, 400))]
    print('协议自检 3 通过：坏帧拒绝 + 字节流重同步')

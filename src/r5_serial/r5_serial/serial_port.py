#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""最小串口层（纯标准库 os/termios/select，零第三方依赖）。

为什么不用 pyserial：目标环境（Ubuntu 22.04 + ROS Humble）默认不带
pyserial 且当前机器无法 pip 安装。串口配置总共就这几行，自己写反而
把波特率/原始模式这些"黑盒"摊开，对学习嵌入式通信更有价值。

对 pty（虚拟串口）与真实 tty（CH340 → /dev/ttyUSBx）统一处理：
raw 模式（无回显/无行缓冲/无流控）、8N1、设定波特率。
pty 的波特率无物理意义，但 termios 设置不会报错，代码路径完全一致
——这就是"虚拟/真实零差别切换"的关键。
"""

import os
import select
import termios
import time

BAUD_CONST = {
    9600: termios.B9600, 19200: termios.B19200, 38400: termios.B38400,
    57600: termios.B57600, 115200: termios.B115200, 230400: termios.B230400,
    460800: termios.B460800, 921600: termios.B921600,
}


class SerialPort:
    def __init__(self, path, baud=230400):
        self.path = path
        # O_NOCTTY：不要把这个串口当成我们的控制终端（防止 Ctrl-C 串扰）
        self.fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            attrs = termios.tcgetattr(self.fd)
            # Linux 返回 7 元素：[iflag, oflag, cflag, lflag, ispeed, ospeed, cc]
            ifd, ofd, cfd, lfd, isp, osp, cc = attrs
            speed = BAUD_CONST.get(baud)
            if speed is not None:
                isp = isp & ~termios.CBAUD
                osp = osp & ~termios.CBAUD
                isp |= speed
                osp |= speed
            # raw 模式：关回显/关规范模式/关信号/8N1/使能接收
            lfd &= ~(termios.ECHO | termios.ICANON | termios.ISIG
                     | termios.IEXTEN)
            ifd &= ~(termios.IXON | termios.IXOFF | termios.IXANY
                     | termios.ICRNL | termios.INLCR)
            ofd &= ~termios.OPOST
            cfd &= ~(termios.CSIZE | termios.PARENB | termios.CSTOPB)
            cfd |= termios.CS8
            cfd |= (termios.CLOCAL | termios.CREAD)
            cfd &= ~termios.CRTSCTS
            # VMIN=0 / VTIME=0：非规范读不阻塞（读超时由 select 控制）
            cc[termios.VMIN] = 0
            cc[termios.VTIME] = 0
            termios.tcsetattr(
                self.fd, termios.TCSANOW,
                [ifd, ofd, cfd, lfd, isp, osp, cc])
        except termios.error:
            pass  # 某些 pty 实现不支持全部属性，忽略
        self.stats = {'tx': 0, 'rx': 0}

    def write(self, data):
        n = os.write(self.fd, data)
        self.stats['tx'] += n
        return n

    def read(self, timeout=0.02, max_bytes=512):
        """带超时读取。返回 bytes（可能为空）。"""
        r, _, _ = select.select([self.fd], [], [], timeout)
        if not r:
            return b''
        data = os.read(self.fd, max_bytes)
        self.stats['rx'] += len(data)
        return data

    def drain(self, quiet=0.05):
        """清空接收缓冲（打开端口后丢弃残留字节，避免旧帧污染）。"""
        deadline = time.monotonic() + quiet
        while time.monotonic() < deadline:
            if not self.read(timeout=0.01):
                break

    def close(self):
        try:
            os.close(self.fd)
        except OSError:
            pass

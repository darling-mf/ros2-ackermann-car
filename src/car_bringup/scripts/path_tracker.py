#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""三合一路径跟踪对比实验台：Pure Pursuit / Stanley / LQR。

绕过 Nav2 的 controller_server，直接订阅 /odom、发布 /cmd_vel，
用同一条路径、同一速度公平对比三种经典横向控制器的跟踪误差。

约定（全文统一，改符号前先读这段）：
  路径坐标系：切线 t̂ 沿行进方向，左法线 n̂ = rot90(t̂)。
  e   = (p_car − p_near)·n̂   >0 表示车在路径【左侧】
  Δψ  = θ_car − θ_path        >0 表示车头朝向路径【左侧】
  δ   >0 表示左转（对应 ω = v·tanδ/L > 0，逆时针）
  ⇒ e>0 要右转（δ<0），Δψ>0 要右转（δ<0），所有修正项都带负号。

用法：
  ros2 run car_bringup path_tracker.py --tracker lqr --path line   --offset 0.3
  ros2 run car_bringup path_tracker.py --tracker pp  --path sine   --amp 0.3
"""

import argparse
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

MAX_STEER = 0.235  # 与 URDF / controllers.yaml 一致的单前轮机械限位


def wrap(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Path:
    """折线路径 + 预计算切线/弧长/曲率 + 带进度窗口的投影。"""

    def __init__(self, pts, wheelbase):
        self.p = np.array(pts)                       # (n, 2)
        d = np.diff(self.p, axis=0)
        self.s = np.concatenate(([0.0], np.cumsum(np.hypot(d[:, 0], d[:, 1]))))
        self.theta = np.unwrap(np.arctan2(d[:, 1], d[:, 0]))  # 每段切线角 (n-1)
        self.theta = np.append(self.theta, self.theta[-1])    # 补到 n 个
        # 曲率 = 切线角对弧长的一阶差分（中心差分平滑）
        self.kappa = np.zeros(len(self.s))
        for i in range(1, len(self.s) - 1):
            ds = self.s[i + 1] - self.s[i - 1]
            if ds > 1e-6:
                self.kappa[i] = (self.theta[i + 1] - self.theta[i - 1]) / ds
        self.wheelbase = wheelbase
        self._idx = 0                                # 进度指针（单调前进）

    def project(self, x, y):
        """带进度窗口的最近点投影，返回 (s, e, dpsi, kappa, theta_path)。"""
        n = len(self.s)
        lo = max(0, self._idx - 60)
        hi = min(n - 1, self._idx + 150)
        seg = self.p[lo:hi + 1]
        d2 = (seg[:, 0] - x) ** 2 + (seg[:, 1] - y) ** 2
        j = int(np.argmin(d2)) + lo
        # 子采样精度：投影到相邻线段
        i0 = max(0, j - 1)
        i1 = min(n - 1, j)
        a, b = self.p[i0], self.p[i1]
        ab = b - a
        L2 = ab @ ab
        t = 0.0 if L2 < 1e-9 else float(np.clip(((np.array([x, y]) - a) @ ab) / L2, 0.0, 1.0))
        pn = a + t * ab
        s = self.s[i0] + t * (self.s[i1] - self.s[i0])
        th = self.theta[i0] + t * (self.theta[i1] - self.theta[i0])
        kap = self.kappa[i0] + t * (self.kappa[i1] - self.kappa[i0])
        # e>0：车在路径左侧（n̂ = 左法线）
        e = -(x - pn[0]) * math.sin(th) + (y - pn[1]) * math.cos(th)
        self._idx = j
        return s, e, th, kap

    def point_ahead(self, s_look):
        """弧长 s_look 处的路径点（用于 Pure Pursuit 前视点）。"""
        i = int(np.searchsorted(self.s, s_look))
        i = min(max(i, 1), len(self.s) - 1)
        t = (s_look - self.s[i - 1]) / max(self.s[i] - self.s[i - 1], 1e-9)
        t = min(max(t, 0.0), 1.0)
        return self.p[i - 1] + t * (self.p[i] - self.p[i - 1])

    @property
    def total_s(self):
        return self.s[-1]


def make_line(pose, offset, length=3.0, ds=0.02):
    """直线：起点在车左侧 offset 米处，方向沿车头。offset>0 → 车初值在路径右侧。"""
    x0, y0, th0 = pose
    nx, ny = -math.sin(th0), math.cos(th0)          # 车头左法线
    sx, sy = x0 + offset * nx, y0 + offset * ny
    tx, ty = math.cos(th0), math.sin(th0)
    n = int(length / ds) + 1
    return [(sx + i * ds * tx, sy + i * ds * ty) for i in range(n)]


def make_sine(pose, amp, wl, length=4.0, ds=0.02):
    """正弦：起点在车上（e=0），横向 = amp·sin(2π s / wl)。测试连续曲率跟随。"""
    x0, y0, th0 = pose
    tx, ty = math.cos(th0), math.sin(th0)
    nx, ny = -math.sin(th0), math.cos(th0)
    n = int(length / ds) + 1
    pts = []
    for i in range(n):
        s = i * ds
        lat = amp * math.sin(2.0 * math.pi * s / wl)
        pts.append((x0 + s * tx + lat * nx, y0 + s * ty + lat * ny))
    return pts


def solve_dare(A, B, Q, R, iters=800, eps=1e-10):
    """离散 Riccati 迭代，返回 K（u = −Kx）。LQR 的全部数学就在这 8 行。

    P = Q + AᵀPA − (AᵀPB)(R+BᵀPB)⁻¹(BᵀPA)
    维度核对：AᵀPB 是 (2×1)，(R+BᵀPB)⁻¹ 是 (1×1)，BᵀPA 是 (1×2)，
    乘起来才回到 (2×2)。转置放错一边 numpy 会当场报 shape 错误
    ——“shape 即类型”，矩阵代码的第一道防线。
    """
    P = Q.copy()
    for _ in range(iters):
        BtPA = B.T @ P @ A                       # (1×2)
        Ric = np.linalg.solve(R + B.T @ P @ B, BtPA)   # (1×2)
        Pn = Q + A.T @ P @ A - BtPA.T @ Ric      # (2×1)@(1×2) → (2×2)
        if np.max(np.abs(Pn - P)) < eps:
            P = Pn
            break
        P = Pn
    return np.linalg.solve(R + B.T @ P @ B, B.T @ P @ A)


class Tracker:
    """基类：子类实现 steer(e, dpsi, kappa, v, extra) → δ [rad]。"""

    def __init__(self, args):
        self.args = args

    def steer(self, e, dpsi, kappa, v, **extra):
        raise NotImplementedError


class PurePursuit(Tracker):
    """前视点追踪：δ = atan(2L·sinα / l_d)。α 为前视点在车体系的角度。

    特点：只有一个调参量 l_d（前视距离）。l_d 小→贴线但易振荡；
    l_d 大→平滑但切弯。误差定义在参考点上（本实验用后轴等效点），
    对横向误差的收敛没有解析保证，纯几何。
    """

    def steer(self, e, dpsi, kappa, v, path=None, s_now=0.0, pose=None, **kw):
        l_d = self.args.lookahead
        target = path.point_ahead(s_now + l_d)
        dx = target[0] - pose[0]
        dy = target[1] - pose[1]
        alpha = wrap(math.atan2(dy, dx) - pose[2])   # 车体系目标角
        return math.atan2(2.0 * self.args.wheelbase * math.sin(alpha), l_d)


class Stanley(Tracker):
    """Stanley：δ = Δψ + atan(k·e_f / v)，误差定义在【前轴】上。

    特点：几何+误差反馈，有收敛性分析（k 足够大时 e→0）；
    致命弱点：v→0 时 atan(k·e/v) 增益发散 → 低速抖振/饱和。
    本实验对 v 下限 0.05 整流，饱和交给 MAX_STEER——
    “低速增益发散”是 Stanley 出名的缺陷，留到对比表里看。
    """

    def steer(self, e, dpsi, kappa, v, path=None, s_now=0.0, pose=None, **kw):
        L = self.args.wheelbase
        # 前轴位置 = 后轴(车体参考) + L·朝向；本实验 pose 取车体参考点
        fx = pose[0] + L * math.cos(pose[2])
        fy = pose[1] + L * math.sin(pose[2])
        _, e_f, th_path, _ = path.project(fx, fy)
        v_eff = max(v, 0.05)
        return -(dpsi + math.atan2(self.args.k_stanley * e_f, v_eff))


class LQRTracker(Tracker):
    """LQR：运动学误差模型 [e, Δψ]，δ = −K·x + 前馈 atan(L·κ)。

    线性化（小角度）离散模型（设计速度 v、周期 dt 固定）：
      e⁺   = e + v·dt·Δψ                     （车头偏左 → 横向误差向左增长）
      Δψ⁺  = Δψ + (v·dt/L)·δ − v·dt·κ        （转向和路径曲率都改变航向差）
    A=[[1, v·dt],[0,1]], B=[[0],[v·dt/L]]，解 DARE 得 K。
    特点：Q/R 显式权衡"收敛快慢 vs 控制平顺"，有最优性框架；
    局限：只在设计速度附近最优（实车会按 v 增益调度）。
    """

    def __init__(self, args, v, dt):
        super().__init__(args)
        L, w = args.wheelbase, args.wheelbase
        A = np.array([[1.0, v * dt], [0.0, 1.0]])
        B = np.array([[0.0], [v * dt / L]])
        Q = np.diag([args.q_e, args.q_psi])
        R = np.array([[args.r_delta]])
        self.K = solve_dare(A, B, Q, R)
        print(f'[LQR] K = {self.K.round(4).tolist()}  '
              f'(Q=diag({args.q_e},{args.q_psi}), R={args.r_delta}, v={v})')

    def steer(self, e, dpsi, kappa, v, **kw):
        u = -self.K @ np.array([e, dpsi])
        ff = math.atan(self.args.wheelbase * kappa)    # 曲率前馈（自行车稳态）
        return float(u[0]) + self.args.ff_gain * ff


TRACKERS = {'pp': PurePursuit, 'stanley': Stanley, 'lqr': LQRTracker}


def main():
    p = argparse.ArgumentParser(description='PP/Stanley/LQR 路径跟踪对比')
    p.add_argument('--tracker', choices=TRACKERS, default='lqr')
    p.add_argument('--path', choices=['line', 'sine'], default='line')
    p.add_argument('--v', type=float, default=0.3, help='巡航速度 m/s')
    p.add_argument('--offset', type=float, default=0.3,
                   help='line：路径起点在车左侧的偏移 m')
    p.add_argument('--amp', type=float, default=0.3, help='sine 横向幅值 m')
    p.add_argument('--wavelength', type=float, default=3.0, help='sine 波长 m')
    p.add_argument('--wheelbase', type=float, default=0.112)
    p.add_argument('--lookahead', type=float, default=0.35, help='PP 前视距离 m')
    p.add_argument('--k-stanley', type=float, default=1.0, help='Stanley 增益')
    p.add_argument('--q-e', type=float, default=25.0, help='LQR 横向误差权重')
    p.add_argument('--q-psi', type=float, default=4.0, help='LQR 航向误差权重')
    p.add_argument('--r-delta', type=float, default=25.0, help='LQR 转角代价')
    p.add_argument('--ff-gain', type=float, default=1.0, help='LQR 曲率前馈增益')
    p.add_argument('--rate', type=float, default=30.0, help='控制频率 Hz')
    p.add_argument('--timeout', type=float, default=25.0, help='单次实验超时 s')
    args = p.parse_args()

    rclpy.init()
    node = Node(f'{args.tracker}_tracker')
    state = {'odom': None}
    node.create_subscription(Odometry, '/odom',
                             lambda m: state.__setitem__('odom', m), 20)
    pub = node.create_publisher(Twist, '/cmd_vel', 10)

    def spin(sec):
        end = time.monotonic() + sec
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.01)

    def publish(v, wz):
        m = Twist()
        m.linear.x = v
        m.angular.z = wz
        pub.publish(m)

    def stop():
        for _ in range(30):
            publish(0.0, 0.0)
            spin(0.02)

    spin(1.0)
    # DDS 发现可能要几秒（尤其守护进程刚重启时），带超时等待而不是死等 1 秒
    t_wait = time.monotonic()
    while state['odom'] is None and time.monotonic() - t_wait < 5.0:
        spin(0.1)
    if state['odom'] is None:
        print('错误：没收到 /odom，先确认控制器 active')
        return 1

    odom0 = state['odom']
    pose0 = (odom0.pose.pose.position.x, odom0.pose.pose.position.y,
             yaw_of(odom0.pose.pose.orientation))
    if args.path == 'line':
        pts = make_line(pose0, args.offset)
    else:
        pts = make_sine(pose0, args.amp, args.wavelength)
    path = Path(pts, args.wheelbase)

    dt = 1.0 / args.rate
    tracker = TRACKERS[args.tracker](args) if args.tracker != 'lqr' \
        else LQRTracker(args, args.v, dt)

    print(f'实验: tracker={args.tracker} path={args.path} v={args.v} '
          f'总长={path.total_s:.2f}m')
    samples = []          # (t, s, e, dpsi)
    t0 = time.monotonic()
    done = False
    while time.monotonic() - t0 < args.timeout and not done:
        o = state['odom']
        if o is None:
            spin(dt)
            continue
        x, y = o.pose.pose.position.x, o.pose.pose.position.y
        th = yaw_of(o.pose.pose.orientation)
        v_meas = o.twist.twist.linear.x

        s, e, th_path, kap = path.project(x, y)
        dpsi = wrap(th - th_path)

        if s >= path.total_s - 0.10:
            done = True
            publish(0.0, 0.0)
            break

        delta = tracker.steer(e, dpsi, kap, v_meas, path=path, s_now=s,
                              pose=(x, y, th))
        delta = max(-MAX_STEER, min(MAX_STEER, delta))
        wz = args.v * math.tan(delta) / args.wheelbase
        publish(args.v, wz)
        samples.append((time.monotonic() - t0, s, e, dpsi))
        spin(dt)

    stop()

    # ============ 指标统计 ============
    if not samples:
        print('无数据')
        return 1
    completed = done
    arr = np.array([(t, s, e, dpsi) for t, s, e, dpsi in samples])
    t_arr, s_arr, e_arr, psi_arr = arr.T

    # 收敛时间：|e| 连续 0.5s < 0.05 的首个时刻（仅 line 有意义）
    settle = None
    hold = 0
    for t, s_, e_, _ in samples:
        if abs(e_) < 0.05:
            hold += dt
            if hold >= 0.5 and settle is None:
                settle = t - 0.5
        else:
            hold = 0

    mask = s_arr > 0.5                      # 跳过初始收敛段看"稳态跟踪"
    if mask.any():
        rms_e = float(np.sqrt(np.mean(e_arr[mask] ** 2)))
        max_e = float(np.max(np.abs(e_arr[mask])))
        rms_psi = float(np.sqrt(np.mean(psi_arr[mask] ** 2)))
    else:
        rms_e = max_e = rms_psi = float('nan')
    init_e = abs(samples[0][2])

    print('\n===== 跟踪指标 =====')
    print(f'完成: {"是" if completed else "否(超时)"} | 用时 {t_arr[-1]:.1f}s | '
          f'初始|e|={init_e:.3f}m')
    if settle is not None:
        print(f'收敛时间(|e|<0.05 持续0.5s): {settle:.2f}s')
    print(f'稳态(s>0.5m): RMS|e|={rms_e:.4f} m | max|e|={max_e:.4f} m | '
          f'RMS|Δψ|={math.degrees(rms_psi):.2f}°')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

#ifndef ACKERMANN_H
#define ACKERMANN_H

/**
 * 阿克曼转向几何（纯数学，无硬件依赖）+ 舵机标定参数化。
 *
 * 修正厂家固件 ax_kinematics.c:107 的单位问题（tan(β*0.01745) 把弧度
 * 又当角度转了一次），并把手感未知的舵机映射改为【可标定参数】：
 *   servo_deg = gain * inner_wheel_deg + offset_deg
 * gain/offset 由 S8 实测（给定若干舵角，测量前轮实际角度后拟合），
 * 不再迷信厂家的多项式系数。
 *
 * 约定：vx>0 前进；wz>0 左转；β>0 左转；右侧轮为外侧。
 */

#include <stdint.h>

typedef struct
{
  float axle_base;      /* 轴距 H [m]，实测（固件 0.146 / 手册 0.112） */
  float wheel_base;     /* 轮距 W [m]，实测（固件 0.160 / 手册 0.164） */
  float min_radius;     /* 最小转弯半径 [m]，实测（固件 0.35 / 手册 0.55） */
  float servo_gain;     /* 舵机角 = gain * 内轮角(deg) + offset */
  float servo_offset;   /* [deg]，含机械零位 */
} akm_cfg_t;

/* 由 (vx mm/s, wz mrad/s) 求虚拟中心转角 β [mrad]；含最小半径钳位。 */
int16_t akm_w_to_beta_mrad(int16_t vx_mm_s, int16_t wz_mrad_s,
                           const akm_cfg_t *cfg);

/* 由 β [rad] 求内侧前轮转角 [deg]（左转内侧=左轮，右转内侧=右轮）。 */
float akm_inner_wheel_deg(float beta_rad, const akm_cfg_t *cfg);

/* 由内轮角 [deg] 求舵机指令（0.1° 单位，固件 AX_SERVO_S3_SetAngle 入参）。 */
int16_t akm_servo_cmd_d10(float inner_deg, const akm_cfg_t *cfg);

#endif /* ACKERMANN_H */

#ifndef SAFETY_H
#define SAFETY_H

/**
 * 速度指令看门狗（纯逻辑，无硬件依赖）。
 *
 * 背景：厂家固件 ax_uart_pi.c 收到 0x50 直接写 RobotV_TG，没有任何超时
 * 逻辑——ROS 断线/崩溃后，车会保持最后速度一直跑。S6 必须在下位机补
 * 这个安全闭环（上位机 serial_bridge 的 0.5s 看门狗只是兜底）。
 *
 * 用法：Robot_Task（20ms）里每周期调用 safety_allow，返回 0 时把
 * 目标速度清零并停止电机。
 */

#include <stdint.h>

typedef struct
{
  uint16_t timeout_ms;   /* 建议 300ms */
  uint32_t last_cmd_ms;  /* 最后一次有效速度帧的时间戳 */
  uint8_t  ever_valid;   /* 上电后是否收到过指令 */
} safety_t;

void safety_init(safety_t *s, uint16_t timeout_ms);

/* 收到合法 0x50 帧时调用（喂狗）。 */
void safety_feed(safety_t *s, uint32_t now_ms);

/* 返回 1=允许运动；0=超时/从未收到指令，必须停车。 */
uint8_t safety_allow(const safety_t *s, uint32_t now_ms);

#endif /* SAFETY_H */

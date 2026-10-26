#ifndef XPROTOCOL_H
#define XPROTOCOL_H

/**
 * X-Protocol 编解码（与厂家固件 ax_uart_pi.c 逐行对照，纯 C、无硬件依赖）。
 * 帧：AA 55 | LEN | ID | DATA(大端) | CK
 *     LEN = len(DATA) + 5；CK = 前面所有字节累加的低 8 位。
 *
 * 用法（UART4 中断里逐字节喂）：
 *   uint8_t id, data[60], n;
 *   if (xproto_feed(&p, byte, &id, data, &n)) { on_frame(id, data, n); }
 * 配套单元测试：test/test_host.c（PC 上 gcc 即可跑）。
 */

#include <stdint.h>

#define XPROTO_HEAD1 0xAAu
#define XPROTO_HEAD2 0x55u
#define XPROTO_MAX_PAYLOAD 50u
#define XPROTO_MAX_FRAME 60u

/* 帧号（固件 ax_robot.h / robot.h） */
#define X_ID_UTX_DATA 0x10u     /* STM32 -> ROS：50Hz 综合数据 */
#define X_ID_URX_VEL 0x50u      /* ROS -> STM32：vx,vy,vw */
#define X_ID_URX_IMU_CAL 0x51u  /* ROS -> STM32：IMU 零偏校准 */
#define X_ID_URX_AKM_OFFSET 0x5Fu /* ROS -> STM32：舵机零偏 */

typedef struct
{
  uint8_t state;                  /* 0 等 AA / 1 等 55 / 2 等 LEN / 3 收体 */
  uint8_t buf[XPROTO_MAX_FRAME];
  uint8_t idx;                    /* 已写入 buf 的字节数 */
  uint8_t ck;                     /* 累加校验和 */
  uint16_t frames;                /* 统计：好帧 */
  uint16_t bad_ck;                /* 统计：坏帧 */
  uint16_t resync;                /* 统计：重同步次数 */
} xproto_t;

void xproto_init(xproto_t *p);

/* 喂一个字节；解析出完整帧时返回 1，并填 id/data/len。 */
int xproto_feed(xproto_t *p, uint8_t byte,
                uint8_t *id, uint8_t *data, uint8_t *len);

/* 打包一帧；out 至少 len+5 字节。返回总帧长。 */
uint8_t xproto_pack(uint8_t id, const uint8_t *data, uint8_t len, uint8_t *out);

#endif /* XPROTOCOL_H */

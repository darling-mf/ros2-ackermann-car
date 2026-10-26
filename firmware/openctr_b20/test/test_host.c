/* 主机单元测试：gcc 编译即可跑，不需要任何硬件。
 * 编译命令见 firmware/openctr_b20/README.md（src 目录下所有 .c 一起编）。
 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ackermann.h"
#include "safety.h"
#include "xprotocol.h"

static void test_official_example(void)
{
  /* 固件文档示例：AA 55 0B 01 03 E8 FC 18 00 0A 14 = (1000, -1000, 10) */
  uint8_t data[6] = {0x03, 0xE8, 0xFC, 0x18, 0x00, 0x0A};
  uint8_t out[16];
  uint8_t n = xproto_pack(0x01, data, 6, out);
  const uint8_t expect[] = {0xAA, 0x55, 0x0B, 0x01, 0x03, 0xE8,
                            0xFC, 0x18, 0x00, 0x0A, 0x14};
  assert(n == sizeof(expect));
  assert(memcmp(out, expect, sizeof(expect)) == 0);
  printf("PASS 官方示例帧逐字节一致\n");
}

static void test_roundtrip(void)
{
  uint8_t payload[20];
  for (uint8_t i = 0; i < 20; i++) { payload[i] = i; }
  uint8_t frame[32];
  uint8_t n = xproto_pack(X_ID_UTX_DATA, payload, 20, frame);

  xproto_t p;
  xproto_init(&p);
  uint8_t id = 0, data[50], len = 0;
  int got = 0;
  for (uint8_t i = 0; i < n; i++)
  {
    if (xproto_feed(&p, frame[i], &id, data, &len)) { got = 1; }
  }
  assert(got && id == X_ID_UTX_DATA && len == 20);
  assert(memcmp(data, payload, 20) == 0);
  printf("PASS 0x10 20 字节 pack/parse 回环\n");
}

static void test_bad_frame_resync(void)
{
  uint8_t vx[6] = {0x00, 0xC8, 0x00, 0x00, 0x01, 0x90}; /* 200,0,400 */
  uint8_t good[16];
  uint8_t n = xproto_pack(X_ID_URX_VEL, vx, 6, good);
  uint8_t bad[16];
  memcpy(bad, good, n);
  bad[7] ^= 0xFF;

  xproto_t p;
  xproto_init(&p);
  uint8_t id = 0, data[50], len = 0;
  int got = 0;
  for (uint8_t i = 0; i < n; i++) { xproto_feed(&p, bad[i], &id, data, &len); }
  for (uint8_t i = 0; i < n; i++)
  {
    if (xproto_feed(&p, good[i], &id, data, &len)) { got = 1; }
  }
  assert(p.bad_ck == 1 && got == 1 && p.frames == 1);
  printf("PASS 坏帧拒绝 + 字节流重同步\n");
}

static void test_safety(void)
{
  safety_t s;
  safety_init(&s, 300);
  assert(safety_allow(&s, 0) == 0);   /* 上电未收到指令：禁止运动 */
  safety_feed(&s, 1000);
  assert(safety_allow(&s, 1200) == 1);
  assert(safety_allow(&s, 1300) == 1);
  assert(safety_allow(&s, 1301) == 0); /* 超过 300ms：停车 */
  safety_feed(&s, 5000);
  assert(safety_allow(&s, 5299) == 1);
  printf("PASS 看门狗 300ms 边界与恢复\n");
}

static void test_ackermann(void)
{
  akm_cfg_t cfg = {0.146f, 0.160f, 0.35f, 1.0f, 0.0f};
  /* vx=0.3, wz=0.6 → R=0.5m；β=atan(0.146/0.5)=0.2843 rad */
  int16_t beta_mrad = akm_w_to_beta_mrad(300, 600, &cfg);
  float beta = beta_mrad / 1000.0f;
  assert(fabsf(beta - 0.2843f) < 0.002f);

  /* 内轮角 = atan(0.146/(0.5-0.08)) = 0.3352 rad = 19.20 deg */
  float inner = akm_inner_wheel_deg(beta, &cfg);
  assert(fabsf(inner - 19.2f) < 0.2f);

  /* 舵机指令（gain=1, offset=0）应为 192（0.1°） */
  assert(abs(akm_servo_cmd_d10(inner, &cfg) - 192) <= 2);

  /* 最小半径钳位：vx=1.0, wz=6.28 → R=0.159 < 0.35 → 钳到 0.35 */
  int16_t beta_lim = akm_w_to_beta_mrad(1000, 6280, &cfg);
  assert(beta_lim > 0 && beta_lim < 500);
  printf("PASS 阿克曼 β/内轮角/舵机指令/最小半径钳位\n");
}

int main(void)
{
  test_official_example();
  test_roundtrip();
  test_bad_frame_resync();
  test_safety();
  test_ackermann();
  printf("\n全部 S6 主机测试通过\n");
  return 0;
}

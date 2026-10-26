#include "ackermann.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

int16_t akm_w_to_beta_mrad(int16_t vx_mm_s, int16_t wz_mrad_s,
                           const akm_cfg_t *cfg)
{
  if (vx_mm_s == 0) { return 0; }          /* 阿克曼不能原地转 */
  float vx = (float)vx_mm_s / 1000.0f;
  float wz = (float)wz_mrad_s / 1000.0f;
  float r = vx / wz;                        /* 有符号转弯半径 */
  float rmin = cfg->min_radius;

  if (r > 0.0f && r < rmin) { r = rmin; }
  if (r < 0.0f && r > -rmin) { r = -rmin; }

  float beta = atanf(cfg->axle_base / r);
  return (int16_t)(beta * 1000.0f);
}

float akm_inner_wheel_deg(float beta_rad, const akm_cfg_t *cfg)
{
  float half_w = cfg->wheel_base * 0.5f;
  float tan_b = tanf(beta_rad);
  if (fabsf(tan_b) < 1e-6f) { return 0.0f; }

  /* R 为后轴中心到瞬心的距离；内侧轮半径 = |R| − W/2 */
  float r = cfg->axle_base / tan_b;
  float r_abs = fabsf(r);
  float r_inner = r_abs - half_w;
  if (r_inner < 1e-3f) { r_inner = 1e-3f; }  /* 防除零（几何上不应发生） */
  float delta = atanf(cfg->axle_base / r_inner);
  /* 右转时内轮在右侧，角度符号与 β 相反 */
  if (r < 0.0f) { delta = -delta; }
  return delta * (float)(180.0 / M_PI);
}

int16_t akm_servo_cmd_d10(float inner_deg, const akm_cfg_t *cfg)
{
  float servo_deg = cfg->servo_gain * inner_deg + cfg->servo_offset;
  return (int16_t)lroundf(servo_deg * 10.0f);
}

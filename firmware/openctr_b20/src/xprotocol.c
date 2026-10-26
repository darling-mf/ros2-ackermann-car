#include "xprotocol.h"

void xproto_init(xproto_t *p)
{
  p->state = 0u;
  p->idx = 0u;
  p->ck = 0u;
  p->frames = 0u;
  p->bad_ck = 0u;
  p->resync = 0u;
}

int xproto_feed(xproto_t *p, uint8_t byte,
                uint8_t *id, uint8_t *data, uint8_t *len)
{
  switch (p->state)
  {
    case 0:
      if (byte == XPROTO_HEAD1) { p->state = 1u; }
      else { p->resync++; }
      break;
    case 1:
      if (byte == XPROTO_HEAD2) { p->state = 2u; }
      else { p->state = 0u; p->resync++; }
      break;
    case 2:
      /* LEN 合法范围：5(空载荷) .. 60 */
      if (byte >= 5u && byte <= XPROTO_MAX_FRAME)
      {
        p->buf[0] = XPROTO_HEAD1;
        p->buf[1] = XPROTO_HEAD2;
        p->buf[2] = byte;
        p->idx = 3u;
        p->ck = (uint8_t)(XPROTO_HEAD1 + XPROTO_HEAD2 + byte);
        p->state = 3u;
      }
      else { p->state = 0u; p->resync++; }
      break;
    default: /* 收 ID/DATA/CK */
      if (p->idx < (uint8_t)(p->buf[2] - 1u))
      {
        p->buf[p->idx++] = byte;
        p->ck = (uint8_t)(p->ck + byte);
      }
      else
      {
        p->state = 0u;
        if (byte == p->ck)
        {
          p->frames++;
          *id = p->buf[3];
          *len = (uint8_t)(p->buf[2] - 5u);
          for (uint8_t i = 0u; i < *len; i++) { data[i] = p->buf[4u + i]; }
          return 1;
        }
        p->bad_ck++;
      }
      break;
  }
  return 0;
}

uint8_t xproto_pack(uint8_t id, const uint8_t *data, uint8_t len, uint8_t *out)
{
  uint8_t total = (uint8_t)(len + 5u);
  uint8_t ck = 0u;
  out[0] = XPROTO_HEAD1;
  out[1] = XPROTO_HEAD2;
  out[2] = total;
  out[3] = id;
  for (uint8_t i = 0u; i < len; i++) { out[4u + i] = data[i]; }
  for (uint8_t i = 0u; i < (uint8_t)(4u + len); i++) { ck = (uint8_t)(ck + out[i]); }
  out[4u + len] = ck;
  return total;
}

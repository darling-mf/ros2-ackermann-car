#include "safety.h"

void safety_init(safety_t *s, uint16_t timeout_ms)
{
  s->timeout_ms = timeout_ms;
  s->last_cmd_ms = 0u;
  s->ever_valid = 0u;
}

void safety_feed(safety_t *s, uint32_t now_ms)
{
  s->last_cmd_ms = now_ms;
  s->ever_valid = 1u;
}

uint8_t safety_allow(const safety_t *s, uint32_t now_ms)
{
  if (!s->ever_valid) { return 0u; }
  return (uint32_t)(now_ms - s->last_cmd_ms) <= s->timeout_ms ? 1u : 0u;
}

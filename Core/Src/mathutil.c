#include "mathutil.h"

uint16_t MathUtil_CalcGCD(uint16_t a, uint16_t b) {
  while (b != 0) {
    uint16_t t = b;
    b = a % b;
    a = t;
  }
  return a;
}

float MathUtil_CalcRatioFloat(uint16_t num, uint16_t den) {
  if (den == 0) return 0.0f;
  uint32_t q = ((uint32_t) num << 16) / den;
  uint32_t rem = ((uint32_t) num << 16) % den;
  uint32_t frac = ((uint32_t) rem << 16) / den;
  return (float) q * (1.0f / 65536.0f) + (float) frac * (1.0f / 4294967296.0f);
}

q12_t MathUtil_FloatToQ12(float val) {
  q12_t q = {
      .raw = (int32_t)(val >= 0.0f ? (val * 4096.0f + 0.5f) : (val * 4096.0f - 0.5f))
  };
  return q;
}

q16_t MathUtil_FloatToQ16(float val) {
  q16_t q = {
      .raw = (int32_t)(val >= 0.0f ? (val * 65536.0f + 0.5f) : (val * 65536.0f - 0.5f))
  };
  return q;
}

float MathUtil_Q12ToFloat(q12_t q) {
  return (float) q.raw * (1.0f / 4096.0f);
}

float MathUtil_Q16ToFloat(q16_t q) {
  return (float) q.raw * (1.0f / 65536.0f);
}

q12_t MathUtil_CalcRatioQ12(uint16_t num, uint16_t den) {
  if (den == 0) return (q12_t){ .raw = 0 };
  return (q12_t){ .raw = (int32_t)(((uint32_t) num << 12) / den) };
}

q16_t MathUtil_CalcRatioQ16(uint16_t num, uint16_t den) {
  if (den == 0) return (q16_t){ .raw = 0 };
  return (q16_t){ .raw = (int32_t)(((uint32_t) num << 16) / den) };
}

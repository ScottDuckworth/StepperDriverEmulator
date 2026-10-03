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

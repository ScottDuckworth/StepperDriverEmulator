#include "mathutil.h"

#include <stdio.h>

uint16_t MathUtil_CalcGCD(uint16_t a, uint16_t b) {
  while (b != 0) {
    uint16_t t = b;
    b = a % b;
    a = t;
  }
  return a;
}

q12_t MathUtil_CalcRatioQ12(uint16_t num, uint16_t den) {
  if (den == 0) return (q12_t){ .raw = 0 };
  return (q12_t){ .raw = (int32_t)(((uint32_t) num << 12) / den) };
}

q16_t MathUtil_CalcRatioQ16(uint16_t num, uint16_t den) {
  if (den == 0) return (q16_t){ .raw = 0 };
  return (q16_t){ .raw = (int32_t)(((uint32_t) num << 16) / den) };
}

bool MathUtil_StrToQ12(const char* str, q12_t* out) {
  if (!str || !out) return false;
  while (*str == ' ' || *str == '\t') str++;
  if (*str == '\0') return false;

  bool negative = false;
  if (*str == '-') {
    negative = true;
    str++;
  } else if (*str == '+') {
    str++;
  }

  if (*str == '\0') return false;

  int32_t int_part = 0;
  bool has_digits = false;
  while (*str >= '0' && *str <= '9') {
    has_digits = true;
    int_part = int_part * 10 + (*str - '0');
    str++;
  }

  uint32_t frac_part = 0;
  uint32_t denom = 1;
  if (*str == '.') {
    str++;
    while (*str >= '0' && *str <= '9') {
      has_digits = true;
      if (denom < 10000U) {
        frac_part = frac_part * 10U + (uint32_t)(*str - '0');
        denom *= 10U;
      }
      str++;
    }
  }

  if (!has_digits) return false;
  while (*str == ' ' || *str == '\t' || *str == '\r' || *str == '\n') str++;
  if (*str != '\0') return false;

  uint32_t frac_q12 = 0;
  if (denom > 1) {
    frac_q12 = ((frac_part * 4096U) + (denom / 2U)) / denom;
  }

  int32_t raw = (int_part << 12) + (int32_t) frac_q12;
  out->raw = negative ? -raw : raw;
  return true;
}

bool MathUtil_StrToQ16(const char* str, q16_t* out) {
  if (!str || !out) return false;
  while (*str == ' ' || *str == '\t') str++;
  if (*str == '\0') return false;

  bool negative = false;
  if (*str == '-') {
    negative = true;
    str++;
  } else if (*str == '+') {
    str++;
  }

  if (*str == '\0') return false;

  int32_t int_part = 0;
  bool has_digits = false;
  while (*str >= '0' && *str <= '9') {
    has_digits = true;
    int_part = int_part * 10 + (*str - '0');
    str++;
  }

  uint32_t frac_part = 0;
  uint32_t denom = 1;
  if (*str == '.') {
    str++;
    while (*str >= '0' && *str <= '9') {
      has_digits = true;
      if (denom < 10000U) {
        frac_part = frac_part * 10U + (uint32_t)(*str - '0');
        denom *= 10U;
      }
      str++;
    }
  }

  if (!has_digits) return false;
  while (*str == ' ' || *str == '\t' || *str == '\r' || *str == '\n') str++;
  if (*str != '\0') return false;

  uint32_t frac_q16 = 0;
  if (denom > 1) {
    frac_q16 = ((frac_part * 65536U) + (denom / 2U)) / denom;
  }

  int32_t raw = (int_part << 16) + (int32_t) frac_q16;
  out->raw = negative ? -raw : raw;
  return true;
}

void MathUtil_FormatQ12(char* buf, size_t buf_sz, q12_t val, uint8_t decimals) {
  if (!buf || buf_sz == 0) return;
  if (decimals > 4) decimals = 4;

  bool negative = (val.raw < 0);
  uint32_t abs_raw = (uint32_t)(negative ? -val.raw : val.raw);
  uint32_t int_part = abs_raw >> 12;
  uint32_t rem = abs_raw & 0xFFFU;

  uint32_t mult = 10000U;
  if (decimals == 1) mult = 10U;
  else if (decimals == 2) mult = 100U;
  else if (decimals == 3) mult = 1000U;

  uint32_t frac_part = ((rem * mult) + 2048U) >> 12;
  if (frac_part >= mult) {
    int_part++;
    frac_part -= mult;
  }

  if (decimals == 0) {
    snprintf(buf, buf_sz, "%s%lu", negative ? "-" : "", (unsigned long) int_part);
  } else {
    char fmt[16];
    snprintf(fmt, sizeof(fmt), "%%s%%lu.%%0%ulu", (unsigned) decimals);
    snprintf(buf, buf_sz, fmt, negative ? "-" : "", (unsigned long) int_part, (unsigned long) frac_part);
  }
}

void MathUtil_FormatQ16(char* buf, size_t buf_sz, q16_t val, uint8_t decimals) {
  if (!buf || buf_sz == 0) return;
  if (decimals > 4) decimals = 4;

  bool negative = (val.raw < 0);
  uint32_t abs_raw = (uint32_t)(negative ? -val.raw : val.raw);
  uint32_t int_part = abs_raw >> 16;
  uint32_t rem = abs_raw & 0xFFFFU;

  uint32_t mult = 10000U;
  if (decimals == 1) mult = 10U;
  else if (decimals == 2) mult = 100U;
  else if (decimals == 3) mult = 1000U;

  uint32_t frac_part = ((rem * mult) + 32768U) >> 16;
  if (frac_part >= mult) {
    int_part++;
    frac_part -= mult;
  }

  if (decimals == 0) {
    snprintf(buf, buf_sz, "%s%lu", negative ? "-" : "", (unsigned long) int_part);
  } else {
    char fmt[16];
    snprintf(fmt, sizeof(fmt), "%%s%%lu.%%0%ulu", (unsigned) decimals);
    snprintf(buf, buf_sz, fmt, negative ? "-" : "", (unsigned long) int_part, (unsigned long) frac_part);
  }
}

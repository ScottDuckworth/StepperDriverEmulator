#include "mathutil.h"

#include <stdio.h>
#include <string.h>

uint16_t MathUtil_GCD(uint16_t a, uint16_t b) {
  while (b != 0) {
    uint16_t t = b;
    b = a % b;
    a = t;
  }
  return a;
}

q12_t MathUtil_RatioQ12(uint16_t num, uint16_t den) {
  if (den == 0) return (q12_t){ .raw = 0 };
  return (q12_t){ .raw = (int32_t)(((uint32_t) num << 12) / den) };
}

q16_t MathUtil_RatioQ16(uint16_t num, uint16_t den) {
  if (den == 0) return (q16_t){ .raw = 0 };
  return (q16_t){ .raw = (int32_t)(((uint32_t) num << 16) / den) };
}

/* Parse an 8-bit unsigned integer from a decimal string. */
bool MathUtil_ParseU8(const char* str, uint8_t* out) {
  if (!out) return false;
  uint32_t val;
  if (!MathUtil_ParseU32(str, &val) || val > 255U) return false;
  *out = (uint8_t) val;
  return true;
}

/* Parse a 16-bit unsigned integer from a decimal string. */
bool MathUtil_ParseU16(const char* str, uint16_t* out) {
  if (!out) return false;
  uint32_t val;
  if (!MathUtil_ParseU32(str, &val) || val > 65535U) return false;
  *out = (uint16_t) val;
  return true;
}

/* Parse a 32-bit unsigned integer from a decimal string. */
bool MathUtil_ParseU32(const char* str, uint32_t* out) {
  if (!str || !out) return false;
  while (*str == ' ' || *str == '\t') str++;
  if (*str == '\0') return false;

  if (*str == '+') {
    str++;
  }
  if (*str < '0' || *str > '9') return false;

  uint32_t val = 0;
  const uint32_t cutoff = 429496729U; /* UINT32_MAX / 10 */
  const uint32_t cutlim = 5U;

  while (*str >= '0' && *str <= '9') {
    uint32_t digit = (uint32_t)(*str - '0');
    if (val > cutoff || (val == cutoff && digit > cutlim)) {
      return false;
    }
    val = val * 10U + digit;
    str++;
  }

  while (*str == ' ' || *str == '\t' || *str == '\r' || *str == '\n') str++;
  if (*str != '\0') return false;

  *out = val;
  return true;
}

/* Parse a 32-bit signed integer from a decimal string. */
bool MathUtil_ParseI32(const char* str, int32_t* out) {
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
  if (*str < '0' || *str > '9') return false;

  uint32_t val = 0;
  const uint32_t cutoff = 214748364U; /* 2147483647 / 10 */
  const uint32_t cutlim = negative ? 8U : 7U;

  while (*str >= '0' && *str <= '9') {
    uint32_t digit = (uint32_t)(*str - '0');
    if (val > cutoff || (val == cutoff && digit > cutlim)) {
      return false;
    }
    val = val * 10U + digit;
    str++;
  }

  while (*str == ' ' || *str == '\t' || *str == '\r' || *str == '\n') str++;
  if (*str != '\0') return false;

  if (negative) {
    if (val == 2147483648U) {
      *out = INT32_MIN;
    } else {
      *out = -(int32_t) val;
    }
  } else {
    *out = (int32_t) val;
  }
  return true;
}

/* Parse a 64-bit signed integer from a decimal string. */
bool MathUtil_ParseI64(const char* str, int64_t* out) {
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
  if (*str < '0' || *str > '9') return false;

  uint64_t val = 0;
  const uint64_t cutoff = 922337203685477580ULL; /* INT64_MAX / 10 */
  const uint32_t cutlim = negative ? 8U : 7U;

  while (*str >= '0' && *str <= '9') {
    uint32_t digit = (uint32_t)(*str - '0');
    if (val > cutoff || (val == cutoff && digit > cutlim)) {
      return false;
    }
    val = val * 10ULL + digit;
    str++;
  }

  while (*str == ' ' || *str == '\t' || *str == '\r' || *str == '\n') str++;
  if (*str != '\0') return false;

  if (negative) {
    if (val == 9223372036854775808ULL) {
      *out = INT64_MIN;
    } else {
      *out = -(int64_t) val;
    }
  } else {
    *out = (int64_t) val;
  }
  return true;
}

/* Parse a Q12 fixed-point value from a decimal string. */
bool MathUtil_ParseQ12(const char* str, q12_t* out) {
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

/* Parse a Q16 fixed-point value from a decimal string. */
bool MathUtil_ParseQ16(const char* str, q16_t* out) {
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

/* Format an 8-bit unsigned integer into a decimal string (min buffer: 4 bytes). */
size_t MathUtil_FormatU8(char* buf, size_t buf_sz, uint8_t val) {
  return MathUtil_FormatU32(buf, buf_sz, (uint32_t) val);
}

/* Format a 16-bit unsigned integer into a decimal string (min buffer: 6 bytes). */
size_t MathUtil_FormatU16(char* buf, size_t buf_sz, uint16_t val) {
  return MathUtil_FormatU32(buf, buf_sz, (uint32_t) val);
}

/* Format a 32-bit unsigned integer into a decimal string (min buffer: 11 bytes). */
size_t MathUtil_FormatU32(char* buf, size_t buf_sz, uint32_t val) {
  if (!buf || buf_sz == 0) return 0;

  char temp[12];
  size_t len = 0;
  if (val == 0) {
    temp[len++] = '0';
  } else {
    while (val > 0) {
      temp[len++] = (char)('0' + (val % 10U));
      val /= 10U;
    }
  }

  if (len + 1 > buf_sz) {
    buf[0] = '\0';
    return 0;
  }

  for (size_t i = 0; i < len; i++) {
    buf[i] = temp[len - 1 - i];
  }
  buf[len] = '\0';
  return len;
}

/* Format a 32-bit signed integer into a decimal string (min buffer: 12 bytes). */
size_t MathUtil_FormatI32(char* buf, size_t buf_sz, int32_t val) {
  if (!buf || buf_sz == 0) return 0;

  bool negative = (val < 0);
  uint32_t mag = negative ? (uint32_t)(-(val + 1)) + 1U : (uint32_t) val;

  char temp[12];
  size_t digits = 0;
  if (mag == 0) {
    temp[digits++] = '0';
  } else {
    while (mag > 0) {
      temp[digits++] = (char)('0' + (mag % 10U));
      mag /= 10U;
    }
  }

  size_t total_len = digits + (negative ? 1 : 0);
  if (total_len + 1 > buf_sz) {
    buf[0] = '\0';
    return 0;
  }

  size_t pos = 0;
  if (negative) {
    buf[pos++] = '-';
  }
  for (size_t i = 0; i < digits; i++) {
    buf[pos++] = temp[digits - 1 - i];
  }
  buf[pos] = '\0';
  return total_len;
}

/* Format a 64-bit signed integer into a decimal string without 64-bit division (min buffer: 21 bytes). */
size_t MathUtil_FormatI64(char* buf, size_t buf_sz, int64_t val) {
  if (!buf || buf_sz == 0) return 0;

  bool negative = (val < 0);
  uint64_t mag;
  if (negative) {
    mag = (uint64_t)(-(val + 1)) + 1ULL;
  } else {
    mag = (uint64_t) val;
  }

  char digits[24];
  int d_cnt = 0;
  if (mag == 0) {
    digits[d_cnt++] = '0';
  } else {
    uint16_t w[4];
    w[0] = (uint16_t)(mag & 0xFFFFULL);
    w[1] = (uint16_t)((mag >> 16) & 0xFFFFULL);
    w[2] = (uint16_t)((mag >> 32) & 0xFFFFULL);
    w[3] = (uint16_t)((mag >> 48) & 0xFFFFULL);
    while (w[0] | w[1] | w[2] | w[3]) {
      uint32_t rem = 0;
      for (int i = 3; i >= 0; i--) {
        uint32_t cur = (rem << 16) | w[i];
        w[i] = (uint16_t)(cur / 10);
        rem = cur % 10;
      }
      digits[d_cnt++] = (char)('0' + rem);
    }
  }

  size_t total_len = (size_t) d_cnt + (negative ? 1 : 0);
  if (total_len + 1 > buf_sz) {
    buf[0] = '\0';
    return 0;
  }

  size_t pos = 0;
  if (negative) {
    buf[pos++] = '-';
  }
  while (d_cnt > 0) {
    buf[pos++] = digits[--d_cnt];
  }
  buf[pos] = '\0';
  return total_len;
}

/* Format a Q12 fixed-point value into a decimal string (min buffer: 13 bytes for up to 4 decimals). */
size_t MathUtil_FormatQ12(char* buf, size_t buf_sz, q12_t val, uint8_t decimals) {
  if (!buf || buf_sz == 0) return 0;
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

  int written;
  if (decimals == 0) {
    written = snprintf(buf, buf_sz, "%s%lu", negative ? "-" : "", (unsigned long) int_part);
  } else {
    char fmt[16];
    snprintf(fmt, sizeof(fmt), "%%s%%lu.%%0%ulu", (unsigned) decimals);
    written = snprintf(buf, buf_sz, fmt, negative ? "-" : "", (unsigned long) int_part, (unsigned long) frac_part);
  }

  if (written < 0 || (size_t) written >= buf_sz) {
    return strlen(buf);
  }
  return (size_t) written;
}

/* Format a Q16 fixed-point value into a decimal string (min buffer: 12 bytes for up to 4 decimals). */
size_t MathUtil_FormatQ16(char* buf, size_t buf_sz, q16_t val, uint8_t decimals) {
  if (!buf || buf_sz == 0) return 0;
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

  int written;
  if (decimals == 0) {
    written = snprintf(buf, buf_sz, "%s%lu", negative ? "-" : "", (unsigned long) int_part);
  } else {
    char fmt[16];
    snprintf(fmt, sizeof(fmt), "%%s%%lu.%%0%ulu", (unsigned) decimals);
    written = snprintf(buf, buf_sz, fmt, negative ? "-" : "", (unsigned long) int_part, (unsigned long) frac_part);
  }

  if (written < 0 || (size_t) written >= buf_sz) {
    return strlen(buf);
  }
  return (size_t) written;
}

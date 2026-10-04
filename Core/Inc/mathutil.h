#ifndef MATHUTIL_H
#define MATHUTIL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/*
 * Strongly typed fixed-point representation structures.
 * Encapsulating scaled integers inside structs prevents mixing up
 * integer counts, velocities, and Q12 / Q16 fixed-point scales at compile-time.
 */
typedef struct {
  int32_t raw;
} q12_t;

typedef struct {
  int32_t raw;
} q16_t;

/*
 * MathUtil_CalcGCD:
 * Computes the Greatest Common Divisor using the Euclidean algorithm.
 * Handles zero values gracefully (GCD(0, x) = x, GCD(0, 0) = 0).
 */
uint16_t MathUtil_CalcGCD(uint16_t a, uint16_t b);

/*
 * Decimal string parsing and formatting for Q12 / Q16 fixed-point numbers.
 * Allows zero-float CLI input parsing and output reporting.
 */
bool MathUtil_StrToQ12(const char* str, q12_t* out);
bool MathUtil_StrToQ16(const char* str, q16_t* out);
void MathUtil_FormatQ12(char* buf, size_t buf_sz, q12_t val, uint8_t decimals);
void MathUtil_FormatQ16(char* buf, size_t buf_sz, q16_t val, uint8_t decimals);

/*
 * Fixed-point raw and integer constructor/conversion helpers:
 */
static inline q12_t MathUtil_FromRawQ12(int32_t raw) {
  q12_t q = { .raw = raw };
  return q;
}

static inline q16_t MathUtil_FromRawQ16(int32_t raw) {
  q16_t q = { .raw = raw };
  return q;
}

static inline q12_t MathUtil_IntToQ12(int32_t val) {
  q12_t q = { .raw = val << 12 };
  return q;
}

static inline int32_t MathUtil_Q12ToInt(q12_t q) {
  return (q.raw + 2048) >> 12;
}

static inline q16_t MathUtil_IntToQ16(int32_t val) {
  q16_t q = { .raw = val << 16 };
  return q;
}

static inline int32_t MathUtil_Q16ToInt(q16_t q) {
  return (q.raw + 32768) >> 16;
}

/*
 * Fixed-point ratio helpers:
 * Computes (num / den) in Q12 and Q16 formats without floats.
 */
q12_t MathUtil_CalcRatioQ12(uint16_t num, uint16_t den);
q16_t MathUtil_CalcRatioQ16(uint16_t num, uint16_t den);

/*
 * Fixed-point multiplication helpers:
 * Executes 32-bit fixed-point multiplication on Cortex-M0.
 * Operands must fit within signed 32-bit limits so intermediate products
 * do not overflow 32 bits, executing via single-cycle muls instructions.
 */

/* Multiplies an integer by a Q12 scale factor, returning an integer. */
static inline int32_t MathUtil_MulQ12(int32_t a, q12_t b) {
  return (a * b.raw) >> 12;
}

/* Multiplies two Q12 scale factors, returning a Q12 scale factor. */
static inline q12_t MathUtil_MulQ12_Q12(q12_t a, q12_t b) {
  q12_t q = { .raw = (a.raw * b.raw) >> 12 };
  return q;
}

/* Multiplies an integer by a Q16 scale factor, returning an integer. */
static inline int32_t MathUtil_MulQ16(int32_t a, q16_t b) {
  return (a * b.raw) >> 16;
}

/* Multiplies two Q16 scale factors, returning a Q16 scale factor. */
static inline q16_t MathUtil_MulQ16_Q16(q16_t a, q16_t b) {
  q16_t q = { .raw = (int32_t)(((int64_t) a.raw * b.raw) >> 16) };
  return q;
}


/*
 * Integer clamp utility:
 * Clamps value between min_val and max_val.
 */
static inline int32_t MathUtil_ClampI32(int32_t val, int32_t min_val, int32_t max_val) {
  if (val < min_val) return min_val;
  if (val > max_val) return max_val;
  return val;
}

#ifdef __cplusplus
}
#endif

#endif /* MATHUTIL_H */

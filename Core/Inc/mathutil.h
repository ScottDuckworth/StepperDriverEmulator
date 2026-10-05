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
 * MathUtil_GCD:
 * Computes the Greatest Common Divisor using the Euclidean algorithm.
 * Handles zero values gracefully (GCD(0, x) = x, GCD(0, 0) = 0).
 */
uint16_t MathUtil_GCD(uint16_t a, uint16_t b);

/*
 * Decimal string parsing and formatting for integer and fixed-point types.
 * Provides zero-float, division-free CLI input parsing and output reporting.
 */

/* Parse an 8-bit unsigned integer from a decimal string. Returns true on success, false on error/overflow. */
bool MathUtil_ParseU8(const char* str, uint8_t* out);

/* Parse a 16-bit unsigned integer from a decimal string. Returns true on success, false on error/overflow. */
bool MathUtil_ParseU16(const char* str, uint16_t* out);

/* Parse a 32-bit unsigned integer from a decimal string. Returns true on success, false on error/overflow. */
bool MathUtil_ParseU32(const char* str, uint32_t* out);

/* Parse a 32-bit signed integer from a decimal string. Returns true on success, false on error/overflow. */
bool MathUtil_ParseI32(const char* str, int32_t* out);

/* Parse a 64-bit signed integer from a decimal string. Returns true on success, false on error/overflow. */
bool MathUtil_ParseI64(const char* str, int64_t* out);

/* Parse a Q12 fixed-point value from a decimal string. Returns true on success, false on error/overflow. */
bool MathUtil_ParseQ12(const char* str, q12_t* out);

/* Parse a Q16 fixed-point value from a decimal string. Returns true on success, false on error/overflow. */
bool MathUtil_ParseQ16(const char* str, q16_t* out);

/* Format an 8-bit unsigned integer into a decimal string (min buffer: 4 bytes). Returns characters written (excluding null terminator). */
size_t MathUtil_FormatU8(char* buf, size_t buf_sz, uint8_t val);

/* Format a 16-bit unsigned integer into a decimal string (min buffer: 6 bytes). Returns characters written (excluding null terminator). */
size_t MathUtil_FormatU16(char* buf, size_t buf_sz, uint16_t val);

/* Format a 32-bit unsigned integer into a decimal string (min buffer: 11 bytes). Returns characters written (excluding null terminator). */
size_t MathUtil_FormatU32(char* buf, size_t buf_sz, uint32_t val);

/* Format a 32-bit signed integer into a decimal string (min buffer: 12 bytes). Returns characters written (excluding null terminator). */
size_t MathUtil_FormatI32(char* buf, size_t buf_sz, int32_t val);

/* Format a 64-bit signed integer into a decimal string without 64-bit division (min buffer: 21 bytes). Returns characters written. */
size_t MathUtil_FormatI64(char* buf, size_t buf_sz, int64_t val);

/* Format a Q12 fixed-point value into a decimal string (min buffer: 13 bytes for up to 4 decimals). Returns characters written (excluding null terminator). */
size_t MathUtil_FormatQ12(char* buf, size_t buf_sz, q12_t val, uint8_t decimals);

/* Format a Q16 fixed-point value into a decimal string (min buffer: 12 bytes for up to 4 decimals). Returns characters written (excluding null terminator). */
size_t MathUtil_FormatQ16(char* buf, size_t buf_sz, q16_t val, uint8_t decimals);

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
q12_t MathUtil_RatioQ12(uint16_t num, uint16_t den);
q16_t MathUtil_RatioQ16(uint16_t num, uint16_t den);

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

/*
 * Absolute value utilities:
 */

/* Computes the absolute value of a signed 32-bit integer. */
static inline int32_t MathUtil_AbsI32(int32_t val) {
  return (val < 0) ? -val : val;
}

/* Computes the absolute value of a signed 64-bit integer. */
static inline int64_t MathUtil_AbsI64(int64_t val) {
  return (val < 0) ? -val : val;
}

/* Computes the absolute value of a Q12 fixed-point value. */
static inline q12_t MathUtil_AbsQ12(q12_t val) {
  q12_t q = { .raw = (val.raw < 0) ? -val.raw : val.raw };
  return q;
}

/* Computes the absolute value of a Q16 fixed-point value. */
static inline q16_t MathUtil_AbsQ16(q16_t val) {
  q16_t q = { .raw = (val.raw < 0) ? -val.raw : val.raw };
  return q;
}

#ifdef __cplusplus
}
#endif

#endif /* MATHUTIL_H */

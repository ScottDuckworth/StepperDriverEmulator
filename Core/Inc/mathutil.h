#ifndef MATHUTIL_H
#define MATHUTIL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*
 * MathUtil_CalcGCD:
 * Computes the Greatest Common Divisor using the Euclidean algorithm.
 * Handles zero values gracefully (GCD(0, x) = x, GCD(0, 0) = 0).
 */
uint16_t MathUtil_CalcGCD(uint16_t a, uint16_t b);

/*
 * MathUtil_CalcRatioFloat:
 * Computes (float) num / (float) den using 32-bit integer arithmetic.
 * Employs two-stage fixed-point division (Q16.16 followed by a second Q16
 * remainder stage) to achieve 32-bit fractional resolution, then converts
 * to float via multiplication by power-of-two constants (1/65536.0f and
 * 1/4294967296.0f).
 *
 * This avoids linking the software floating-point division library (__aeabi_fdiv),
 * saving Flash space on Cortex-M0 while preserving exact float accuracy.
 */
float MathUtil_CalcRatioFloat(uint16_t num, uint16_t den);

#ifdef __cplusplus
}
#endif

#endif /* MATHUTIL_H */

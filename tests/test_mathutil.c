#include "unity.h"
#include "mathutil.h"
#include <math.h>

void setUp(void) {}
void tearDown(void) {}

/* --- Greatest Common Divisor (GCD) Tests --- */

void test_mathutil_calc_gcd(void) {
  TEST_ASSERT_EQUAL_UINT16(0, MathUtil_CalcGCD(0, 0));
  TEST_ASSERT_EQUAL_UINT16(5, MathUtil_CalcGCD(0, 5));
  TEST_ASSERT_EQUAL_UINT16(5, MathUtil_CalcGCD(5, 0));
  TEST_ASSERT_EQUAL_UINT16(1000, MathUtil_CalcGCD(1000, 4000));
  TEST_ASSERT_EQUAL_UINT16(8, MathUtil_CalcGCD(200, 1024));
  TEST_ASSERT_EQUAL_UINT16(1, MathUtil_CalcGCD(17, 19));
  TEST_ASSERT_EQUAL_UINT16(60, MathUtil_CalcGCD(360, 60));
  TEST_ASSERT_EQUAL_UINT16(1000, MathUtil_CalcGCD(1000, 1000));
  TEST_ASSERT_EQUAL_UINT16(65535, MathUtil_CalcGCD(65535, 65535));
  TEST_ASSERT_EQUAL_UINT16(65535, MathUtil_CalcGCD(65535, 0));
}

/* --- Decimal String Parsing and Formatting Tests --- */

void test_mathutil_str_to_q12(void) {
  q12_t q;
  TEST_ASSERT_TRUE(MathUtil_StrToQ12("0", &q));
  TEST_ASSERT_EQUAL_INT32(0, q.raw);

  TEST_ASSERT_TRUE(MathUtil_StrToQ12("1", &q));
  TEST_ASSERT_EQUAL_INT32(4096, q.raw);

  TEST_ASSERT_TRUE(MathUtil_StrToQ12("-1", &q));
  TEST_ASSERT_EQUAL_INT32(-4096, q.raw);

  TEST_ASSERT_TRUE(MathUtil_StrToQ12("0.1", &q));
  TEST_ASSERT_EQUAL_INT32(410, q.raw); // 0.1 * 4096 = 409.6 -> 410

  TEST_ASSERT_TRUE(MathUtil_StrToQ12("0.005", &q));
  TEST_ASSERT_EQUAL_INT32(20, q.raw); // 0.005 * 4096 = 20.48 -> 20

  TEST_ASSERT_TRUE(MathUtil_StrToQ12("3.5", &q));
  TEST_ASSERT_EQUAL_INT32(14336, q.raw); // 3.5 * 4096 = 14336

  TEST_ASSERT_TRUE(MathUtil_StrToQ12("  0.25  ", &q));
  TEST_ASSERT_EQUAL_INT32(1024, q.raw);

  TEST_ASSERT_FALSE(MathUtil_StrToQ12("", &q));
  TEST_ASSERT_FALSE(MathUtil_StrToQ12("abc", &q));
  TEST_ASSERT_FALSE(MathUtil_StrToQ12("1.2.3", &q));
  TEST_ASSERT_FALSE(MathUtil_StrToQ12("1.2a", &q));
}

void test_mathutil_str_to_q16(void) {
  q16_t q;
  TEST_ASSERT_TRUE(MathUtil_StrToQ16("0", &q));
  TEST_ASSERT_EQUAL_INT32(0, q.raw);

  TEST_ASSERT_TRUE(MathUtil_StrToQ16("1", &q));
  TEST_ASSERT_EQUAL_INT32(65536, q.raw);

  TEST_ASSERT_TRUE(MathUtil_StrToQ16("-1", &q));
  TEST_ASSERT_EQUAL_INT32(-65536, q.raw);

  TEST_ASSERT_TRUE(MathUtil_StrToQ16("0.25", &q));
  TEST_ASSERT_EQUAL_INT32(16384, q.raw);

  TEST_ASSERT_TRUE(MathUtil_StrToQ16("0.005", &q));
  TEST_ASSERT_EQUAL_INT32(328, q.raw); // 0.005 * 65536 = 327.68 -> 328

  TEST_ASSERT_FALSE(MathUtil_StrToQ16("", &q));
  TEST_ASSERT_FALSE(MathUtil_StrToQ16("xyz", &q));
}

void test_mathutil_format_q12_and_q16(void) {
  char buf[32];

  MathUtil_FormatQ12(buf, sizeof(buf), (q12_t){.raw = 4096}, 4);
  TEST_ASSERT_EQUAL_STRING("1.0000", buf);

  MathUtil_FormatQ12(buf, sizeof(buf), (q12_t){.raw = 410}, 4);
  TEST_ASSERT_EQUAL_STRING("0.1001", buf); // 410 / 4096 = 0.100097... -> 0.1001

  MathUtil_FormatQ12(buf, sizeof(buf), (q12_t){.raw = 14336}, 4);
  TEST_ASSERT_EQUAL_STRING("3.5000", buf);

  MathUtil_FormatQ12(buf, sizeof(buf), (q12_t){.raw = -4096}, 4);
  TEST_ASSERT_EQUAL_STRING("-1.0000", buf);

  MathUtil_FormatQ16(buf, sizeof(buf), (q16_t){.raw = 16384}, 4);
  TEST_ASSERT_EQUAL_STRING("0.2500", buf);

  MathUtil_FormatQ16(buf, sizeof(buf), (q16_t){.raw = 65536}, 4);
  TEST_ASSERT_EQUAL_STRING("1.0000", buf);
}

void test_mathutil_calc_ratio_q12_and_q16(void) {
  TEST_ASSERT_EQUAL_INT32(0, MathUtil_CalcRatioQ12(0, 0).raw);
  TEST_ASSERT_EQUAL_INT32(0, MathUtil_CalcRatioQ12(100, 0).raw);
  TEST_ASSERT_EQUAL_INT32(4096, MathUtil_CalcRatioQ12(1, 1).raw);
  TEST_ASSERT_EQUAL_INT32(16384, MathUtil_CalcRatioQ12(4, 1).raw);
  TEST_ASSERT_EQUAL_INT32(2048, MathUtil_CalcRatioQ12(1, 2).raw);
  TEST_ASSERT_EQUAL_INT32(16384, MathUtil_CalcRatioQ12(4000, 1000).raw);

  TEST_ASSERT_EQUAL_INT32(0, MathUtil_CalcRatioQ16(0, 0).raw);
  TEST_ASSERT_EQUAL_INT32(0, MathUtil_CalcRatioQ16(100, 0).raw);
  TEST_ASSERT_EQUAL_INT32(65536, MathUtil_CalcRatioQ16(1, 1).raw);
  TEST_ASSERT_EQUAL_INT32(262144, MathUtil_CalcRatioQ16(4, 1).raw);
  TEST_ASSERT_EQUAL_INT32(16384, MathUtil_CalcRatioQ16(1, 4).raw);
  TEST_ASSERT_EQUAL_INT32(16384, MathUtil_CalcRatioQ16(1000, 4000).raw);
}

void test_mathutil_mul_q12_and_q16(void) {
  // Q12 multiplication
  // 1.0 (4096) * 1.0 (4096) = 1.0 (4096)
  TEST_ASSERT_EQUAL_INT32(4096, MathUtil_MulQ12(4096, (q12_t){.raw = 4096}));
  // 200,000 (rate) * 1.0 (4096) = 200,000
  TEST_ASSERT_EQUAL_INT32(200000, MathUtil_MulQ12(200000, (q12_t){.raw = 4096}));
  // 200,000 * 0.1 (410) = 20,019 (~20,000)
  TEST_ASSERT_EQUAL_INT32(20019, MathUtil_MulQ12(200000, (q12_t){.raw = 410}));
  // -500 * 4096 = -500
  TEST_ASSERT_EQUAL_INT32(-500, MathUtil_MulQ12(-500, (q12_t){.raw = 4096}));

  // Q12 * Q12 multiplication
  TEST_ASSERT_EQUAL_INT32(4096, MathUtil_MulQ12_Q12((q12_t){.raw = 4096}, (q12_t){.raw = 4096}).raw);
  TEST_ASSERT_EQUAL_INT32(16384, MathUtil_MulQ12_Q12((q12_t){.raw = 4096}, (q12_t){.raw = 16384}).raw);

  // Q16 multiplication (32-bit intermediate product for integer scaling)
  TEST_ASSERT_EQUAL_INT32(30000, MathUtil_MulQ16(30000, (q16_t){.raw = 65536}));
  TEST_ASSERT_EQUAL_INT32(1000, MathUtil_MulQ16(1000, (q16_t){.raw = 65536}));
  TEST_ASSERT_EQUAL_INT32(250, MathUtil_MulQ16(1000, (q16_t){.raw = 16384})); // 1000 * 0.25
  TEST_ASSERT_EQUAL_INT32(-250, MathUtil_MulQ16(-1000, (q16_t){.raw = 16384}));

  // Q16 * Q16 multiplication
  TEST_ASSERT_EQUAL_INT32(65536, MathUtil_MulQ16_Q16((q16_t){.raw = 65536}, (q16_t){.raw = 65536}).raw);

  // Q16 * Q16 multiplication with large numbers (using 64-bit intermediate product)
  // 65536 * 65536 = 4,294,967,296. In Q16: 65536
  TEST_ASSERT_EQUAL_INT32(65536, MathUtil_MulQ16_Q16((q16_t){.raw = 65536}, (q16_t){.raw = 65536}).raw);
  // 4000 * 4000 = 16,000,000. In Q16: 16,000,000 * 16384 = 262,144,000,000 (overflows 32-bit!)
  // MathUtil_MulQ16_Q16 safely computes: (16,000,000 * 16384) >> 16 = 4,000,000
  TEST_ASSERT_EQUAL_INT32(4000000, MathUtil_MulQ16_Q16((q16_t){.raw = 16000000}, (q16_t){.raw = 16384}).raw);

  // Conversion helpers
  TEST_ASSERT_EQUAL_INT32(20480, MathUtil_IntToQ12(5).raw);
  TEST_ASSERT_EQUAL_INT32(5, MathUtil_Q12ToInt((q12_t){.raw = 20480}));
  TEST_ASSERT_EQUAL_INT32(327680, MathUtil_IntToQ16(5).raw);
  TEST_ASSERT_EQUAL_INT32(5, MathUtil_Q16ToInt((q16_t){.raw = 327680}));
}

void test_mathutil_clamp_i32(void) {
  TEST_ASSERT_EQUAL_INT32(10, MathUtil_ClampI32(5, 10, 20));
  TEST_ASSERT_EQUAL_INT32(15, MathUtil_ClampI32(15, 10, 20));
  TEST_ASSERT_EQUAL_INT32(20, MathUtil_ClampI32(25, 10, 20));
  TEST_ASSERT_EQUAL_INT32(-10, MathUtil_ClampI32(-15, -10, 10));
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_mathutil_calc_gcd);
  RUN_TEST(test_mathutil_str_to_q12);
  RUN_TEST(test_mathutil_str_to_q16);
  RUN_TEST(test_mathutil_format_q12_and_q16);
  RUN_TEST(test_mathutil_calc_ratio_q12_and_q16);
  RUN_TEST(test_mathutil_mul_q12_and_q16);
  RUN_TEST(test_mathutil_clamp_i32);
  return UNITY_END();
}


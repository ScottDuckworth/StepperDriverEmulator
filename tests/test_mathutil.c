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

/* --- Fixed-Point Float Ratio Tests --- */

void test_mathutil_calc_ratio_float_div_by_zero(void) {
  TEST_ASSERT_EQUAL_FLOAT(0.0f, MathUtil_CalcRatioFloat(0, 0));
  TEST_ASSERT_EQUAL_FLOAT(0.0f, MathUtil_CalcRatioFloat(100, 0));
  TEST_ASSERT_EQUAL_FLOAT(0.0f, MathUtil_CalcRatioFloat(65535, 0));
}

void test_mathutil_calc_ratio_float_integers(void) {
  TEST_ASSERT_EQUAL_FLOAT(0.0f, MathUtil_CalcRatioFloat(0, 100));
  TEST_ASSERT_EQUAL_FLOAT(1.0f, MathUtil_CalcRatioFloat(1, 1));
  TEST_ASSERT_EQUAL_FLOAT(4.0f, MathUtil_CalcRatioFloat(4, 1));
  TEST_ASSERT_EQUAL_FLOAT(1.0f, MathUtil_CalcRatioFloat(1000, 1000));
  TEST_ASSERT_EQUAL_FLOAT(4.0f, MathUtil_CalcRatioFloat(4000, 1000));
  TEST_ASSERT_EQUAL_FLOAT(100.0f, MathUtil_CalcRatioFloat(10000, 100));
}

void test_mathutil_calc_ratio_float_fractions(void) {
  TEST_ASSERT_EQUAL_FLOAT(0.5f, MathUtil_CalcRatioFloat(1, 2));
  TEST_ASSERT_EQUAL_FLOAT(0.25f, MathUtil_CalcRatioFloat(1, 4));
  TEST_ASSERT_EQUAL_FLOAT(0.125f, MathUtil_CalcRatioFloat(1, 8));
  TEST_ASSERT_EQUAL_FLOAT(0.0625f, MathUtil_CalcRatioFloat(1, 16));
  TEST_ASSERT_EQUAL_FLOAT(0.25f, MathUtil_CalcRatioFloat(1000, 4000));
  TEST_ASSERT_EQUAL_FLOAT(0.1953125f, MathUtil_CalcRatioFloat(200, 1024));
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 5.12f, MathUtil_CalcRatioFloat(1024, 200));
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 5.12f, MathUtil_CalcRatioFloat(128, 25));
}

void test_mathutil_calc_ratio_float_accuracy(void) {
  // Test non-power-of-two fractional ratios against standard IEEE float division
  static const struct {
    uint16_t num;
    uint16_t den;
  } test_cases[] = {
    {1, 3},
    {2, 3},
    {1, 7},
    {5, 7},
    {3, 11},
    {17, 19},
    {200, 1000},
    {800, 1000},
    {1600, 200},
    {200, 1600},
    {2048, 10000},
    {32768, 65535},
    {65535, 32768}
  };

  for (size_t i = 0; i < sizeof(test_cases) / sizeof(test_cases[0]); ++i) {
    float expected = (float) test_cases[i].num / (float) test_cases[i].den;
    float actual = MathUtil_CalcRatioFloat(test_cases[i].num, test_cases[i].den);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, expected, actual);
  }
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_mathutil_calc_gcd);
  RUN_TEST(test_mathutil_calc_ratio_float_div_by_zero);
  RUN_TEST(test_mathutil_calc_ratio_float_integers);
  RUN_TEST(test_mathutil_calc_ratio_float_fractions);
  RUN_TEST(test_mathutil_calc_ratio_float_accuracy);
  return UNITY_END();
}

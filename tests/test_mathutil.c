#include "unity.h"
#include "mathutil.h"
#include <math.h>

void setUp(void) {}
void tearDown(void) {}

/* --- Greatest Common Divisor (GCD) Tests --- */

void test_mathutil_gcd(void) {
  TEST_ASSERT_EQUAL_UINT16(0, MathUtil_GCD(0, 0));
  TEST_ASSERT_EQUAL_UINT16(5, MathUtil_GCD(0, 5));
  TEST_ASSERT_EQUAL_UINT16(5, MathUtil_GCD(5, 0));
  TEST_ASSERT_EQUAL_UINT16(1000, MathUtil_GCD(1000, 4000));
  TEST_ASSERT_EQUAL_UINT16(8, MathUtil_GCD(200, 1024));
  TEST_ASSERT_EQUAL_UINT16(1, MathUtil_GCD(17, 19));
  TEST_ASSERT_EQUAL_UINT16(60, MathUtil_GCD(360, 60));
  TEST_ASSERT_EQUAL_UINT16(1000, MathUtil_GCD(1000, 1000));
  TEST_ASSERT_EQUAL_UINT16(65535, MathUtil_GCD(65535, 65535));
  TEST_ASSERT_EQUAL_UINT16(65535, MathUtil_GCD(65535, 0));
}

/* --- Decimal String Parsing and Formatting Tests --- */

void test_mathutil_parse_unsigned(void) {
  uint8_t u8;
  TEST_ASSERT_TRUE(MathUtil_ParseU8("0", &u8));
  TEST_ASSERT_EQUAL_UINT8(0, u8);
  TEST_ASSERT_TRUE(MathUtil_ParseU8("255", &u8));
  TEST_ASSERT_EQUAL_UINT8(255, u8);
  TEST_ASSERT_TRUE(MathUtil_ParseU8("+42", &u8));
  TEST_ASSERT_EQUAL_UINT8(42, u8);
  TEST_ASSERT_TRUE(MathUtil_ParseU8("  100  \r\n", &u8));
  TEST_ASSERT_EQUAL_UINT8(100, u8);
  TEST_ASSERT_FALSE(MathUtil_ParseU8("256", &u8));
  TEST_ASSERT_FALSE(MathUtil_ParseU8("-1", &u8));
  TEST_ASSERT_FALSE(MathUtil_ParseU8("abc", &u8));
  TEST_ASSERT_FALSE(MathUtil_ParseU8("", &u8));

  uint16_t u16;
  TEST_ASSERT_TRUE(MathUtil_ParseU16("0", &u16));
  TEST_ASSERT_EQUAL_UINT16(0, u16);
  TEST_ASSERT_TRUE(MathUtil_ParseU16("65535", &u16));
  TEST_ASSERT_EQUAL_UINT16(65535, u16);
  TEST_ASSERT_TRUE(MathUtil_ParseU16("+1000", &u16));
  TEST_ASSERT_EQUAL_UINT16(1000, u16);
  TEST_ASSERT_FALSE(MathUtil_ParseU16("65536", &u16));
  TEST_ASSERT_FALSE(MathUtil_ParseU16("-10", &u16));

  uint32_t u32;
  TEST_ASSERT_TRUE(MathUtil_ParseU32("0", &u32));
  TEST_ASSERT_EQUAL_UINT32(0, u32);
  TEST_ASSERT_TRUE(MathUtil_ParseU32("4294967295", &u32));
  TEST_ASSERT_EQUAL_UINT32(4294967295U, u32);
  TEST_ASSERT_TRUE(MathUtil_ParseU32("+12345678", &u32));
  TEST_ASSERT_EQUAL_UINT32(12345678U, u32);
  TEST_ASSERT_FALSE(MathUtil_ParseU32("4294967296", &u32));
  TEST_ASSERT_FALSE(MathUtil_ParseU32("9999999999", &u32));
  TEST_ASSERT_FALSE(MathUtil_ParseU32("-5", &u32));
}

void test_mathutil_parse_signed(void) {
  int32_t i32;
  TEST_ASSERT_TRUE(MathUtil_ParseI32("0", &i32));
  TEST_ASSERT_EQUAL_INT32(0, i32);
  TEST_ASSERT_TRUE(MathUtil_ParseI32("2147483647", &i32));
  TEST_ASSERT_EQUAL_INT32(2147483647, i32);
  TEST_ASSERT_TRUE(MathUtil_ParseI32("-2147483648", &i32));
  TEST_ASSERT_EQUAL_INT32(INT32_MIN, i32);
  TEST_ASSERT_TRUE(MathUtil_ParseI32("+500", &i32));
  TEST_ASSERT_EQUAL_INT32(500, i32);
  TEST_ASSERT_TRUE(MathUtil_ParseI32("  -12345  \t", &i32));
  TEST_ASSERT_EQUAL_INT32(-12345, i32);
  TEST_ASSERT_FALSE(MathUtil_ParseI32("2147483648", &i32));
  TEST_ASSERT_FALSE(MathUtil_ParseI32("-2147483649", &i32));
  TEST_ASSERT_FALSE(MathUtil_ParseI32("abc", &i32));
  TEST_ASSERT_FALSE(MathUtil_ParseI32("", &i32));

  int64_t i64;
  TEST_ASSERT_TRUE(MathUtil_ParseI64("0", &i64));
  TEST_ASSERT_EQUAL_INT64(0, i64);
  TEST_ASSERT_TRUE(MathUtil_ParseI64("9223372036854775807", &i64));
  TEST_ASSERT_EQUAL_INT64(9223372036854775807LL, i64);
  TEST_ASSERT_TRUE(MathUtil_ParseI64("-9223372036854775808", &i64));
  TEST_ASSERT_EQUAL_INT64(-9223372036854775807LL - 1LL, i64);
  TEST_ASSERT_TRUE(MathUtil_ParseI64("+12345678901234", &i64));
  TEST_ASSERT_EQUAL_INT64(12345678901234LL, i64);
  TEST_ASSERT_TRUE(MathUtil_ParseI64("-12345678901234", &i64));
  TEST_ASSERT_EQUAL_INT64(-12345678901234LL, i64);
  TEST_ASSERT_FALSE(MathUtil_ParseI64("9223372036854775808", &i64));
  TEST_ASSERT_FALSE(MathUtil_ParseI64("-9223372036854775809", &i64));
  TEST_ASSERT_FALSE(MathUtil_ParseI64("99999999999999999999", &i64));
  TEST_ASSERT_FALSE(MathUtil_ParseI64("123a", &i64));
  TEST_ASSERT_FALSE(MathUtil_ParseI64("", &i64));
}

void test_mathutil_parse_q12(void) {
  q12_t q;
  TEST_ASSERT_TRUE(MathUtil_ParseQ12("0", &q));
  TEST_ASSERT_EQUAL_INT32(0, q.raw);

  TEST_ASSERT_TRUE(MathUtil_ParseQ12("1", &q));
  TEST_ASSERT_EQUAL_INT32(4096, q.raw);

  TEST_ASSERT_TRUE(MathUtil_ParseQ12("-1", &q));
  TEST_ASSERT_EQUAL_INT32(-4096, q.raw);

  TEST_ASSERT_TRUE(MathUtil_ParseQ12("0.1", &q));
  TEST_ASSERT_EQUAL_INT32(410, q.raw); // 0.1 * 4096 = 409.6 -> 410

  TEST_ASSERT_TRUE(MathUtil_ParseQ12("0.005", &q));
  TEST_ASSERT_EQUAL_INT32(20, q.raw); // 0.005 * 4096 = 20.48 -> 20

  TEST_ASSERT_TRUE(MathUtil_ParseQ12("3.5", &q));
  TEST_ASSERT_EQUAL_INT32(14336, q.raw); // 3.5 * 4096 = 14336

  TEST_ASSERT_TRUE(MathUtil_ParseQ12("  0.25  ", &q));
  TEST_ASSERT_EQUAL_INT32(1024, q.raw);

  TEST_ASSERT_FALSE(MathUtil_ParseQ12("", &q));
  TEST_ASSERT_FALSE(MathUtil_ParseQ12("abc", &q));
  TEST_ASSERT_FALSE(MathUtil_ParseQ12("1.2.3", &q));
  TEST_ASSERT_FALSE(MathUtil_ParseQ12("1.2a", &q));
}

void test_mathutil_parse_q16(void) {
  q16_t q;
  TEST_ASSERT_TRUE(MathUtil_ParseQ16("0", &q));
  TEST_ASSERT_EQUAL_INT32(0, q.raw);

  TEST_ASSERT_TRUE(MathUtil_ParseQ16("1", &q));
  TEST_ASSERT_EQUAL_INT32(65536, q.raw);

  TEST_ASSERT_TRUE(MathUtil_ParseQ16("-1", &q));
  TEST_ASSERT_EQUAL_INT32(-65536, q.raw);

  TEST_ASSERT_TRUE(MathUtil_ParseQ16("0.25", &q));
  TEST_ASSERT_EQUAL_INT32(16384, q.raw);

  TEST_ASSERT_TRUE(MathUtil_ParseQ16("0.005", &q));
  TEST_ASSERT_EQUAL_INT32(328, q.raw); // 0.005 * 65536 = 327.68 -> 328

  TEST_ASSERT_FALSE(MathUtil_ParseQ16("", &q));
  TEST_ASSERT_FALSE(MathUtil_ParseQ16("xyz", &q));
}

void test_mathutil_format_integers(void) {
  char buf[32];

  TEST_ASSERT_EQUAL_UINT32(1, MathUtil_FormatU8(buf, sizeof(buf), 0));
  TEST_ASSERT_EQUAL_STRING("0", buf);
  TEST_ASSERT_EQUAL_UINT32(3, MathUtil_FormatU8(buf, sizeof(buf), 255));
  TEST_ASSERT_EQUAL_STRING("255", buf);

  TEST_ASSERT_EQUAL_UINT32(1, MathUtil_FormatU16(buf, sizeof(buf), 0));
  TEST_ASSERT_EQUAL_STRING("0", buf);
  TEST_ASSERT_EQUAL_UINT32(5, MathUtil_FormatU16(buf, sizeof(buf), 65535));
  TEST_ASSERT_EQUAL_STRING("65535", buf);

  TEST_ASSERT_EQUAL_UINT32(1, MathUtil_FormatU32(buf, sizeof(buf), 0));
  TEST_ASSERT_EQUAL_STRING("0", buf);
  TEST_ASSERT_EQUAL_UINT32(10, MathUtil_FormatU32(buf, sizeof(buf), 4294967295U));
  TEST_ASSERT_EQUAL_STRING("4294967295", buf);

  TEST_ASSERT_EQUAL_UINT32(1, MathUtil_FormatI32(buf, sizeof(buf), 0));
  TEST_ASSERT_EQUAL_STRING("0", buf);
  TEST_ASSERT_EQUAL_UINT32(4, MathUtil_FormatI32(buf, sizeof(buf), -123));
  TEST_ASSERT_EQUAL_STRING("-123", buf);
  TEST_ASSERT_EQUAL_UINT32(10, MathUtil_FormatI32(buf, sizeof(buf), 2147483647));
  TEST_ASSERT_EQUAL_STRING("2147483647", buf);
  TEST_ASSERT_EQUAL_UINT32(11, MathUtil_FormatI32(buf, sizeof(buf), INT32_MIN));
  TEST_ASSERT_EQUAL_STRING("-2147483648", buf);

  TEST_ASSERT_EQUAL_UINT32(1, MathUtil_FormatI64(buf, sizeof(buf), 0));
  TEST_ASSERT_EQUAL_STRING("0", buf);
  TEST_ASSERT_EQUAL_UINT32(4, MathUtil_FormatI64(buf, sizeof(buf), -500));
  TEST_ASSERT_EQUAL_STRING("-500", buf);
  TEST_ASSERT_EQUAL_UINT32(19, MathUtil_FormatI64(buf, sizeof(buf), 9223372036854775807LL));
  TEST_ASSERT_EQUAL_STRING("9223372036854775807", buf);
  TEST_ASSERT_EQUAL_UINT32(20, MathUtil_FormatI64(buf, sizeof(buf), -9223372036854775807LL - 1LL));
  TEST_ASSERT_EQUAL_STRING("-9223372036854775808", buf);

  // Buffer truncation
  char small_buf[3];
  TEST_ASSERT_EQUAL_UINT32(0, MathUtil_FormatU8(small_buf, sizeof(small_buf), 255)); // Needs 4 bytes ("255\0")
}

void test_mathutil_format_q12_and_q16(void) {
  char buf[32];

  TEST_ASSERT_EQUAL_UINT32(6, MathUtil_FormatQ12(buf, sizeof(buf), (q12_t){.raw = 4096}, 4));
  TEST_ASSERT_EQUAL_STRING("1.0000", buf);

  TEST_ASSERT_EQUAL_UINT32(6, MathUtil_FormatQ12(buf, sizeof(buf), (q12_t){.raw = 410}, 4));
  TEST_ASSERT_EQUAL_STRING("0.1001", buf); // 410 / 4096 = 0.100097... -> 0.1001

  TEST_ASSERT_EQUAL_UINT32(6, MathUtil_FormatQ12(buf, sizeof(buf), (q12_t){.raw = 14336}, 4));
  TEST_ASSERT_EQUAL_STRING("3.5000", buf);

  TEST_ASSERT_EQUAL_UINT32(7, MathUtil_FormatQ12(buf, sizeof(buf), (q12_t){.raw = -4096}, 4));
  TEST_ASSERT_EQUAL_STRING("-1.0000", buf);

  TEST_ASSERT_EQUAL_UINT32(6, MathUtil_FormatQ16(buf, sizeof(buf), (q16_t){.raw = 16384}, 4));
  TEST_ASSERT_EQUAL_STRING("0.2500", buf);

  TEST_ASSERT_EQUAL_UINT32(6, MathUtil_FormatQ16(buf, sizeof(buf), (q16_t){.raw = 65536}, 4));
  TEST_ASSERT_EQUAL_STRING("1.0000", buf);
}

void test_mathutil_ratio_q12_and_q16(void) {
  TEST_ASSERT_EQUAL_INT32(0, MathUtil_RatioQ12(0, 0).raw);
  TEST_ASSERT_EQUAL_INT32(0, MathUtil_RatioQ12(100, 0).raw);
  TEST_ASSERT_EQUAL_INT32(4096, MathUtil_RatioQ12(1, 1).raw);
  TEST_ASSERT_EQUAL_INT32(16384, MathUtil_RatioQ12(4, 1).raw);
  TEST_ASSERT_EQUAL_INT32(2048, MathUtil_RatioQ12(1, 2).raw);
  TEST_ASSERT_EQUAL_INT32(16384, MathUtil_RatioQ12(4000, 1000).raw);

  TEST_ASSERT_EQUAL_INT32(0, MathUtil_RatioQ16(0, 0).raw);
  TEST_ASSERT_EQUAL_INT32(0, MathUtil_RatioQ16(100, 0).raw);
  TEST_ASSERT_EQUAL_INT32(65536, MathUtil_RatioQ16(1, 1).raw);
  TEST_ASSERT_EQUAL_INT32(262144, MathUtil_RatioQ16(4, 1).raw);
  TEST_ASSERT_EQUAL_INT32(16384, MathUtil_RatioQ16(1, 4).raw);
  TEST_ASSERT_EQUAL_INT32(16384, MathUtil_RatioQ16(1000, 4000).raw);
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
  RUN_TEST(test_mathutil_gcd);
  RUN_TEST(test_mathutil_parse_unsigned);
  RUN_TEST(test_mathutil_parse_signed);
  RUN_TEST(test_mathutil_parse_q12);
  RUN_TEST(test_mathutil_parse_q16);
  RUN_TEST(test_mathutil_format_integers);
  RUN_TEST(test_mathutil_format_q12_and_q16);
  RUN_TEST(test_mathutil_ratio_q12_and_q16);
  RUN_TEST(test_mathutil_mul_q12_and_q16);
  RUN_TEST(test_mathutil_clamp_i32);
  return UNITY_END();
}


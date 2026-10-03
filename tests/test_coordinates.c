#include "unity.h"
#include "motion_math.h"
#include "emulator_config.h"

void setUp(void) {}
void tearDown(void) {}

void test_calc_gcd(void) {
  TEST_ASSERT_EQUAL_UINT16(0, CalcGCD(0, 0));
  TEST_ASSERT_EQUAL_UINT16(5, CalcGCD(0, 5));
  TEST_ASSERT_EQUAL_UINT16(5, CalcGCD(5, 0));
  TEST_ASSERT_EQUAL_UINT16(1000, CalcGCD(1000, 4000));
  TEST_ASSERT_EQUAL_UINT16(8, CalcGCD(200, 1024));
  TEST_ASSERT_EQUAL_UINT16(1, CalcGCD(17, 19));
  TEST_ASSERT_EQUAL_UINT16(60, CalcGCD(360, 60));
  TEST_ASSERT_EQUAL_UINT16(1000, CalcGCD(1000, 1000));
}

void test_step_delta_integer_ratio_4_to_1(void) {
  int32_t rem = 0;
  uint16_t ratio_spr = 1;
  uint16_t ratio_epr = 4;

  TEST_ASSERT_EQUAL_INT32(0, ConvertStepDeltaToCounts(0, ratio_spr, ratio_epr, &rem));
  TEST_ASSERT_EQUAL_INT32(0, rem);

  TEST_ASSERT_EQUAL_INT32(4, ConvertStepDeltaToCounts(1, ratio_spr, ratio_epr, &rem));
  TEST_ASSERT_EQUAL_INT32(0, rem);

  TEST_ASSERT_EQUAL_INT32(400, ConvertStepDeltaToCounts(100, ratio_spr, ratio_epr, &rem));
  TEST_ASSERT_EQUAL_INT32(0, rem);

  TEST_ASSERT_EQUAL_INT32(-4, ConvertStepDeltaToCounts(-1, ratio_spr, ratio_epr, &rem));
  TEST_ASSERT_EQUAL_INT32(0, rem);

  TEST_ASSERT_EQUAL_INT32(-400, ConvertStepDeltaToCounts(-100, ratio_spr, ratio_epr, &rem));
  TEST_ASSERT_EQUAL_INT32(0, rem);
}

void test_step_delta_fractional_ratio_200_to_1024(void) {
  // 200 spr, 1024 epr -> GCD=8 -> ratio_spr=25, ratio_epr=128 (5.12 counts/step)
  int32_t rem = 0;
  uint16_t ratio_spr = 25;
  uint16_t ratio_epr = 128;

  int64_t total_counts = 0;
  for (int i = 0; i < 25; ++i) {
    total_counts += ConvertStepDeltaToCounts(1, ratio_spr, ratio_epr, &rem);
  }
  // After 25 steps: 25 * 5.12 = 128 counts exactly, remainder 0
  TEST_ASSERT_EQUAL_INT64(128, total_counts);
  TEST_ASSERT_EQUAL_INT32(0, rem);

  // Full revolution: 200 steps = 8 * 25 steps -> 1024 counts
  total_counts = 0;
  rem = 0;
  for (int i = 0; i < 200; ++i) {
    total_counts += ConvertStepDeltaToCounts(1, ratio_spr, ratio_epr, &rem);
  }
  TEST_ASSERT_EQUAL_INT64(1024, total_counts);
  TEST_ASSERT_EQUAL_INT32(0, rem);
}

void test_step_delta_reversals_no_drift(void) {
  // Fractional ratio 25 spr to 128 epr
  int32_t rem = 0;
  uint16_t ratio_spr = 25;
  uint16_t ratio_epr = 128;

  int64_t pos = 0;

  // Forward 10 steps
  for (int i = 0; i < 10; ++i) {
    pos += ConvertStepDeltaToCounts(1, ratio_spr, ratio_epr, &rem);
  }
  TEST_ASSERT_EQUAL_INT64(51, pos); // 10 * 5.12 = 51.2 -> 51 counts
  TEST_ASSERT_EQUAL_INT32(5, rem);  // 10 * 128 = 1280. 1280 % 25 = 5

  // Backward 10 steps (one step at a time)
  for (int i = 0; i < 10; ++i) {
    pos += ConvertStepDeltaToCounts(-1, ratio_spr, ratio_epr, &rem);
  }
  TEST_ASSERT_EQUAL_INT64(0, pos);
  TEST_ASSERT_EQUAL_INT32(0, rem);

  // Ping-pong single steps
  for (int i = 0; i < 50; ++i) {
    pos += ConvertStepDeltaToCounts(1, ratio_spr, ratio_epr, &rem);
    pos += ConvertStepDeltaToCounts(-1, ratio_spr, ratio_epr, &rem);
  }
  TEST_ASSERT_EQUAL_INT64(0, pos);
  TEST_ASSERT_EQUAL_INT32(0, rem);
}

void test_step_delta_microstepping_ratio(void) {
  // 16000 microsteps / rev, 4000 encoder counts -> ratio_spr=4, ratio_epr=1 (0.25 counts/step)
  int32_t rem = 0;
  uint16_t ratio_spr = 4;
  uint16_t ratio_epr = 1;

  TEST_ASSERT_EQUAL_INT32(0, ConvertStepDeltaToCounts(1, ratio_spr, ratio_epr, &rem));
  TEST_ASSERT_EQUAL_INT32(1, rem);

  TEST_ASSERT_EQUAL_INT32(0, ConvertStepDeltaToCounts(1, ratio_spr, ratio_epr, &rem));
  TEST_ASSERT_EQUAL_INT32(2, rem);

  TEST_ASSERT_EQUAL_INT32(0, ConvertStepDeltaToCounts(1, ratio_spr, ratio_epr, &rem));
  TEST_ASSERT_EQUAL_INT32(3, rem);

  TEST_ASSERT_EQUAL_INT32(1, ConvertStepDeltaToCounts(1, ratio_spr, ratio_epr, &rem));
  TEST_ASSERT_EQUAL_INT32(0, rem);

  // Negative delta
  TEST_ASSERT_EQUAL_INT32(-1, ConvertStepDeltaToCounts(-4, ratio_spr, ratio_epr, &rem));
  TEST_ASSERT_EQUAL_INT32(0, rem);
}

void test_step_delta_protection_and_null(void) {
  int32_t rem = 5;

  // Zero ratio_spr guard
  TEST_ASSERT_EQUAL_INT32(0, ConvertStepDeltaToCounts(10, 0, 4, &rem));
  TEST_ASSERT_EQUAL_INT32(5, rem);

  // NULL remainder pointer safe execution
  TEST_ASSERT_EQUAL_INT32(40, ConvertStepDeltaToCounts(10, 1, 4, NULL));
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_calc_gcd);
  RUN_TEST(test_step_delta_integer_ratio_4_to_1);
  RUN_TEST(test_step_delta_fractional_ratio_200_to_1024);
  RUN_TEST(test_step_delta_reversals_no_drift);
  RUN_TEST(test_step_delta_microstepping_ratio);
  RUN_TEST(test_step_delta_protection_and_null);
  return UNITY_END();
}

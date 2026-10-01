#include "unity.h"
#include "motion_math.h"
#include "emulator_config.h"

static EmulatorConfig_t test_config;

void setUp(void) {
  test_config = (EmulatorConfig_t) DEFAULT_EMULATOR_CONFIG;
}

void tearDown(void) {}

void test_step_to_encoder_default_ratio_4_to_1(void) {
  test_config.spr = 1000;
  test_config.epr = 4000;

  TEST_ASSERT_EQUAL_INT32(0, StepToEncoderPositionConfig(&test_config, 0));
  TEST_ASSERT_EQUAL_INT32(1000, StepToEncoderPositionConfig(&test_config, 250));
  TEST_ASSERT_EQUAL_INT32(2000, StepToEncoderPositionConfig(&test_config, 500));
  TEST_ASSERT_EQUAL_INT32(4000, StepToEncoderPositionConfig(&test_config, 1000));
  TEST_ASSERT_EQUAL_INT32(-1000, StepToEncoderPositionConfig(&test_config, -250));
  TEST_ASSERT_EQUAL_INT32(-4000, StepToEncoderPositionConfig(&test_config, -1000));
}

void test_encoder_to_step_default_ratio_4_to_1(void) {
  test_config.spr = 1000;
  test_config.epr = 4000;

  TEST_ASSERT_EQUAL_INT32(0, EncoderToStepPositionConfig(&test_config, 0));
  TEST_ASSERT_EQUAL_INT32(250, EncoderToStepPositionConfig(&test_config, 1000));
  TEST_ASSERT_EQUAL_INT32(500, EncoderToStepPositionConfig(&test_config, 2000));
  TEST_ASSERT_EQUAL_INT32(1000, EncoderToStepPositionConfig(&test_config, 4000));
  TEST_ASSERT_EQUAL_INT32(-250, EncoderToStepPositionConfig(&test_config, -1000));
  TEST_ASSERT_EQUAL_INT32(-1000, EncoderToStepPositionConfig(&test_config, -4000));
}

void test_1_to_1_ratio(void) {
  test_config.spr = 2000;
  test_config.epr = 2000;

  TEST_ASSERT_EQUAL_INT32(0, StepToEncoderPositionConfig(&test_config, 0));
  TEST_ASSERT_EQUAL_INT32(1234, StepToEncoderPositionConfig(&test_config, 1234));
  TEST_ASSERT_EQUAL_INT32(-5678, StepToEncoderPositionConfig(&test_config, -5678));

  TEST_ASSERT_EQUAL_INT32(0, EncoderToStepPositionConfig(&test_config, 0));
  TEST_ASSERT_EQUAL_INT32(1234, EncoderToStepPositionConfig(&test_config, 1234));
  TEST_ASSERT_EQUAL_INT32(-5678, EncoderToStepPositionConfig(&test_config, -5678));
}

void test_non_integer_ratio(void) {
  // 1024 encoder counts / 200 full steps = 5.12 counts per step
  test_config.spr = 200;
  test_config.epr = 1024;

  TEST_ASSERT_EQUAL_INT32(51, StepToEncoderPositionConfig(&test_config, 10)); // 10 * 5.12 = 51.2 -> 51
  TEST_ASSERT_EQUAL_INT32(512, StepToEncoderPositionConfig(&test_config, 100)); // 100 * 5.12 = 512
  TEST_ASSERT_EQUAL_INT32(1024, StepToEncoderPositionConfig(&test_config, 200)); // 200 * 5.12 = 1024
  TEST_ASSERT_EQUAL_INT32(-512, StepToEncoderPositionConfig(&test_config, -100));

  // Reverse conversion
  TEST_ASSERT_EQUAL_INT32(100, EncoderToStepPositionConfig(&test_config, 512));
  TEST_ASSERT_EQUAL_INT32(200, EncoderToStepPositionConfig(&test_config, 1024));
  TEST_ASSERT_EQUAL_INT32(-100, EncoderToStepPositionConfig(&test_config, -512));
}

void test_microstepping_ratio(void) {
  // Microstepping: 16000 microsteps / rev, 4000 encoder counts / rev (4 steps per encoder tick)
  test_config.spr = 16000;
  test_config.epr = 4000;

  TEST_ASSERT_EQUAL_INT32(0, StepToEncoderPositionConfig(&test_config, 0));
  TEST_ASSERT_EQUAL_INT32(0, StepToEncoderPositionConfig(&test_config, 3)); // 3/4 -> 0
  TEST_ASSERT_EQUAL_INT32(1, StepToEncoderPositionConfig(&test_config, 4)); // 4/4 -> 1
  TEST_ASSERT_EQUAL_INT32(1000, StepToEncoderPositionConfig(&test_config, 4000));
  TEST_ASSERT_EQUAL_INT32(-1000, StepToEncoderPositionConfig(&test_config, -4000));

  // Reverse conversion
  TEST_ASSERT_EQUAL_INT32(4, EncoderToStepPositionConfig(&test_config, 1));
  TEST_ASSERT_EQUAL_INT32(4000, EncoderToStepPositionConfig(&test_config, 1000));
  TEST_ASSERT_EQUAL_INT32(-4000, EncoderToStepPositionConfig(&test_config, -1000));
}

void test_zero_and_null_protection(void) {
  // NULL pointer safety
  TEST_ASSERT_EQUAL_INT32(0, StepToEncoderPositionConfig(NULL, 100));
  TEST_ASSERT_EQUAL_INT32(0, EncoderToStepPositionConfig(NULL, 100));

  // Zero spr protection (divide by zero guard)
  test_config.spr = 0;
  test_config.epr = 4000;
  TEST_ASSERT_EQUAL_INT32(0, StepToEncoderPositionConfig(&test_config, 500));

  // Zero epr protection (divide by zero guard)
  test_config.spr = 1000;
  test_config.epr = 0;
  TEST_ASSERT_EQUAL_INT32(0, EncoderToStepPositionConfig(&test_config, 500));
}

void test_large_coordinates_64bit_intermediate(void) {
  test_config.spr = 1000;
  test_config.epr = 4000;

  // 100,000,000 steps * 4000 = 400,000,000,000 (exceeds 32-bit INT32_MAX = 2,147,483,647)
  // But final result 400,000,000 fits in int32_t.
  int32_t large_step = 100000000;
  int32_t expected_encoder = 400000000;
  TEST_ASSERT_EQUAL_INT32(expected_encoder, StepToEncoderPositionConfig(&test_config, large_step));
  TEST_ASSERT_EQUAL_INT32(large_step, EncoderToStepPositionConfig(&test_config, expected_encoder));

  // Negative large coordinates
  TEST_ASSERT_EQUAL_INT32(-expected_encoder, StepToEncoderPositionConfig(&test_config, -large_step));
  TEST_ASSERT_EQUAL_INT32(-large_step, EncoderToStepPositionConfig(&test_config, -expected_encoder));
}

void test_roundtrip_consistency(void) {
  test_config.spr = 1000;
  test_config.epr = 4000;

  for (int32_t step = -5000; step <= 5000; step += 250) {
    int32_t enc = StepToEncoderPositionConfig(&test_config, step);
    int32_t roundtrip_step = EncoderToStepPositionConfig(&test_config, enc);
    TEST_ASSERT_EQUAL_INT32(step, roundtrip_step);
  }
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_step_to_encoder_default_ratio_4_to_1);
  RUN_TEST(test_encoder_to_step_default_ratio_4_to_1);
  RUN_TEST(test_1_to_1_ratio);
  RUN_TEST(test_non_integer_ratio);
  RUN_TEST(test_microstepping_ratio);
  RUN_TEST(test_zero_and_null_protection);
  RUN_TEST(test_large_coordinates_64bit_intermediate);
  RUN_TEST(test_roundtrip_consistency);
  return UNITY_END();
}

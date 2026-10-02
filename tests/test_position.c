#include "unity.h"
#include "position_tracker.h"
#include "emulator_config.h"

static EmulatorConfig_t config;

void setUp(void) {
  config = (EmulatorConfig_t) DEFAULT_EMULATOR_CONFIG;
}

void tearDown(void) {}

void test_step_delta_normal(void) {
  TEST_ASSERT_EQUAL_UINT16(0, CalcStepDelta(500, 500));
  TEST_ASSERT_EQUAL_UINT16(5, CalcStepDelta(105, 100));
  TEST_ASSERT_EQUAL_UINT16(500, CalcStepDelta(1000, 500));
  TEST_ASSERT_EQUAL_UINT16(1, CalcStepDelta(1, 0));
}

void test_step_delta_rollover_16bit(void) {
  // Direct 65535 (0xFFFF) -> 0 wrap: 1 step
  TEST_ASSERT_EQUAL_UINT16(1, CalcStepDelta(0, 65535));

  // 65535 -> 5: (65535->0 = 1) + 5 = 6 steps
  TEST_ASSERT_EQUAL_UINT16(6, CalcStepDelta(5, 65535));

  // 65500 -> 100: (65536 - 65500) + 100 = 136 steps
  TEST_ASSERT_EQUAL_UINT16(136, CalcStepDelta(100, 65500));

  // 60000 -> 10000: (65536 - 60000) + 10000 = 15536 steps
  TEST_ASSERT_EQUAL_UINT16(15536, CalcStepDelta(10000, 60000));
}

void test_accumulate_step_position_forward(void) {
  TEST_ASSERT_EQUAL_INT64(110, AccumulateStepPosition(100, 10, false));
  TEST_ASSERT_EQUAL_INT64(-25, AccumulateStepPosition(-50, 25, false));
  TEST_ASSERT_EQUAL_INT64(0, AccumulateStepPosition(0, 0, false));
  TEST_ASSERT_EQUAL_INT64(1000000, AccumulateStepPosition(999900, 100, false));
  // 64-bit coordinates exceeding 32-bit integer limits (> 2^31 - 1)
  TEST_ASSERT_EQUAL_INT64(5000000010LL, AccumulateStepPosition(5000000000LL, 10, false));
  TEST_ASSERT_EQUAL_INT64(-4999999990LL, AccumulateStepPosition(-5000000000LL, 10, false));
}

void test_accumulate_step_position_reverse(void) {
  TEST_ASSERT_EQUAL_INT64(90, AccumulateStepPosition(100, 10, true));
  TEST_ASSERT_EQUAL_INT64(-75, AccumulateStepPosition(-50, 25, true));
  TEST_ASSERT_EQUAL_INT64(0, AccumulateStepPosition(0, 0, true));
  TEST_ASSERT_EQUAL_INT64(999800, AccumulateStepPosition(999900, 100, true));
  // 64-bit coordinates exceeding 32-bit integer limits (< -2^31)
  TEST_ASSERT_EQUAL_INT64(4999999990LL, AccumulateStepPosition(5000000000LL, 10, true));
  TEST_ASSERT_EQUAL_INT64(-5000000010LL, AccumulateStepPosition(-5000000000LL, 10, true));
}

void test_realign_position_counters(void) {
  config.spr = 1000;
  config.epr = 4000;

  PositionCounters_t pos = {
      .step_pos = 9999,
      .encoder_pos = 9999,
      .step_cnt_prev = 123,
      .step_dcnt = 456,
      .step_reverse = false
  };

  // Realign to encoder position 4000 (1 rev = 1000 steps)
  RealignPositionCounters(&pos, 4000, &config, 5000);
  TEST_ASSERT_EQUAL_INT64(4000, pos.encoder_pos);
  TEST_ASSERT_EQUAL_INT64(1000, pos.step_pos);
  TEST_ASSERT_EQUAL_UINT16(5000, pos.step_cnt_prev);
  TEST_ASSERT_EQUAL_UINT16(0, pos.step_dcnt);

  // Negative encoder position (-2000 counts = -500 steps)
  RealignPositionCounters(&pos, -2000, &config, 1000);
  TEST_ASSERT_EQUAL_INT64(-2000, pos.encoder_pos);
  TEST_ASSERT_EQUAL_INT64(-500, pos.step_pos);
  TEST_ASSERT_EQUAL_UINT16(1000, pos.step_cnt_prev);
  TEST_ASSERT_EQUAL_UINT16(0, pos.step_dcnt);

  // Realign to large 64-bit position (8 billion counts = 2 billion steps)
  RealignPositionCounters(&pos, 8000000000LL, &config, 2000);
  TEST_ASSERT_EQUAL_INT64(8000000000LL, pos.encoder_pos);
  TEST_ASSERT_EQUAL_INT64(2000000000LL, pos.step_pos);

  // Realign to large negative 64-bit position (-12 billion counts = -3 billion steps)
  RealignPositionCounters(&pos, -12000000000LL, &config, 3000);
  TEST_ASSERT_EQUAL_INT64(-12000000000LL, pos.encoder_pos);
  TEST_ASSERT_EQUAL_INT64(-3000000000LL, pos.step_pos);

  // Null safety
  RealignPositionCounters(NULL, 1000, &config, 0);
}

void test_filter_step_blanking_normal_200khz(void) {
  uint32_t accum = 0;
  uint32_t out_period = 0;
  // 200 kHz = 240 ticks @ 48 MHz. Threshold = 168 ticks (3.5 us).
  for (int i = 0; i < 5; i++) {
    uint16_t step = FilterStepWithBlanking(240, 168, &accum, &out_period);
    TEST_ASSERT_EQUAL_UINT16(1, step);
    TEST_ASSERT_EQUAL_UINT32(240, out_period);
    TEST_ASSERT_EQUAL_UINT32(0, accum);
  }
}

void test_filter_step_blanking_glitch_rejection_and_recovery(void) {
  uint32_t accum = 0;
  uint32_t out_period = 0;
  const uint32_t threshold = 168; // 3.5 us

  // Valid initial step at 200 kHz (240 ticks)
  TEST_ASSERT_EQUAL_UINT16(1, FilterStepWithBlanking(240, threshold, &accum, &out_period));
  TEST_ASSERT_EQUAL_UINT32(240, out_period);
  TEST_ASSERT_EQUAL_UINT32(0, accum);

  // Glitch 1: 30 ticks (0.625 us) later -> rejected!
  TEST_ASSERT_EQUAL_UINT16(0, FilterStepWithBlanking(30, threshold, &accum, &out_period));
  TEST_ASSERT_EQUAL_UINT32(30, accum);

  // Glitch 2: 40 ticks (0.833 us) later -> rejected!
  TEST_ASSERT_EQUAL_UINT16(0, FilterStepWithBlanking(40, threshold, &accum, &out_period));
  TEST_ASSERT_EQUAL_UINT32(70, accum);

  // Genuine step arriving 170 ticks after Glitch 2 (total 30 + 40 + 170 = 240 ticks from Step 1)
  TEST_ASSERT_EQUAL_UINT16(1, FilterStepWithBlanking(170, threshold, &accum, &out_period));
  TEST_ASSERT_EQUAL_UINT32(240, out_period);
  TEST_ASSERT_EQUAL_UINT32(0, accum);
}

void test_filter_step_blanking_standstill_primed(void) {
  uint32_t threshold = 168;
  uint32_t accum = threshold; // Primed for standstill
  uint32_t out_period = 0;

  // First step from standstill should trigger immediately regardless of interval
  TEST_ASSERT_EQUAL_UINT16(1, FilterStepWithBlanking(100, threshold, &accum, &out_period));
  TEST_ASSERT_EQUAL_UINT32(268, out_period);
  TEST_ASSERT_EQUAL_UINT32(0, accum);
}

void test_filter_step_blanking_null_safety(void) {
  uint32_t out_period = 0;
  TEST_ASSERT_EQUAL_UINT16(0, FilterStepWithBlanking(240, 168, NULL, &out_period));
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_step_delta_normal);
  RUN_TEST(test_step_delta_rollover_16bit);
  RUN_TEST(test_accumulate_step_position_forward);
  RUN_TEST(test_accumulate_step_position_reverse);
  RUN_TEST(test_realign_position_counters);
  RUN_TEST(test_filter_step_blanking_normal_200khz);
  RUN_TEST(test_filter_step_blanking_glitch_rejection_and_recovery);
  RUN_TEST(test_filter_step_blanking_standstill_primed);
  RUN_TEST(test_filter_step_blanking_null_safety);
  return UNITY_END();
}

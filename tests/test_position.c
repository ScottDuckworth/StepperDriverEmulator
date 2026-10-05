#include "unity.h"
#include "position.h"
#include "emulator_config.h"

static EmulatorConfig_t config;

void setUp(void) {
  config = (EmulatorConfig_t) DEFAULT_EMULATOR_CONFIG;
}

void tearDown(void) {}

/* --- Step-to-Count Ratio Conversion Tests --- */


void test_step_delta_integer_ratio_4_to_1(void) {
  int32_t rem = 0;
  uint16_t ratio_spr = 1;
  uint16_t ratio_epr = 4;

  TEST_ASSERT_EQUAL_INT32(0, Position_ConvertStepDeltaToCounts(0, ratio_spr, ratio_epr, &rem));
  TEST_ASSERT_EQUAL_INT32(0, rem);

  TEST_ASSERT_EQUAL_INT32(4, Position_ConvertStepDeltaToCounts(1, ratio_spr, ratio_epr, &rem));
  TEST_ASSERT_EQUAL_INT32(0, rem);

  TEST_ASSERT_EQUAL_INT32(400, Position_ConvertStepDeltaToCounts(100, ratio_spr, ratio_epr, &rem));
  TEST_ASSERT_EQUAL_INT32(0, rem);

  TEST_ASSERT_EQUAL_INT32(-4, Position_ConvertStepDeltaToCounts(-1, ratio_spr, ratio_epr, &rem));
  TEST_ASSERT_EQUAL_INT32(0, rem);

  TEST_ASSERT_EQUAL_INT32(-400, Position_ConvertStepDeltaToCounts(-100, ratio_spr, ratio_epr, &rem));
  TEST_ASSERT_EQUAL_INT32(0, rem);
}

void test_step_delta_fractional_ratio_200_to_1024(void) {
  // 200 spr, 1024 epr -> GCD=8 -> ratio_spr=25, ratio_epr=128 (5.12 counts/step)
  int32_t rem = 0;
  uint16_t ratio_spr = 25;
  uint16_t ratio_epr = 128;

  int64_t total_counts = 0;
  for (int i = 0; i < 25; ++i) {
    total_counts += Position_ConvertStepDeltaToCounts(1, ratio_spr, ratio_epr, &rem);
  }
  // After 25 steps: 25 * 5.12 = 128 counts exactly, remainder 0
  TEST_ASSERT_EQUAL_INT64(128, total_counts);
  TEST_ASSERT_EQUAL_INT32(0, rem);

  // Full revolution: 200 steps = 8 * 25 steps -> 1024 counts
  total_counts = 0;
  rem = 0;
  for (int i = 0; i < 200; ++i) {
    total_counts += Position_ConvertStepDeltaToCounts(1, ratio_spr, ratio_epr, &rem);
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
    pos += Position_ConvertStepDeltaToCounts(1, ratio_spr, ratio_epr, &rem);
  }
  TEST_ASSERT_EQUAL_INT64(51, pos); // 10 * 5.12 = 51.2 -> 51 counts
  TEST_ASSERT_EQUAL_INT32(5, rem);  // 10 * 128 = 1280. 1280 % 25 = 5

  // Backward 10 steps (one step at a time)
  for (int i = 0; i < 10; ++i) {
    pos += Position_ConvertStepDeltaToCounts(-1, ratio_spr, ratio_epr, &rem);
  }
  TEST_ASSERT_EQUAL_INT64(0, pos);
  TEST_ASSERT_EQUAL_INT32(0, rem);

  // Ping-pong single steps
  for (int i = 0; i < 50; ++i) {
    pos += Position_ConvertStepDeltaToCounts(1, ratio_spr, ratio_epr, &rem);
    pos += Position_ConvertStepDeltaToCounts(-1, ratio_spr, ratio_epr, &rem);
  }
  TEST_ASSERT_EQUAL_INT64(0, pos);
  TEST_ASSERT_EQUAL_INT32(0, rem);
}

void test_step_delta_microstepping_ratio(void) {
  // 16000 microsteps / rev, 4000 encoder counts -> ratio_spr=4, ratio_epr=1 (0.25 counts/step)
  int32_t rem = 0;
  uint16_t ratio_spr = 4;
  uint16_t ratio_epr = 1;

  TEST_ASSERT_EQUAL_INT32(0, Position_ConvertStepDeltaToCounts(1, ratio_spr, ratio_epr, &rem));
  TEST_ASSERT_EQUAL_INT32(1, rem);

  TEST_ASSERT_EQUAL_INT32(0, Position_ConvertStepDeltaToCounts(1, ratio_spr, ratio_epr, &rem));
  TEST_ASSERT_EQUAL_INT32(2, rem);

  TEST_ASSERT_EQUAL_INT32(0, Position_ConvertStepDeltaToCounts(1, ratio_spr, ratio_epr, &rem));
  TEST_ASSERT_EQUAL_INT32(3, rem);

  TEST_ASSERT_EQUAL_INT32(1, Position_ConvertStepDeltaToCounts(1, ratio_spr, ratio_epr, &rem));
  TEST_ASSERT_EQUAL_INT32(0, rem);

  // Negative delta
  TEST_ASSERT_EQUAL_INT32(-1, Position_ConvertStepDeltaToCounts(-4, ratio_spr, ratio_epr, &rem));
  TEST_ASSERT_EQUAL_INT32(0, rem);
}

void test_step_delta_protection_and_null(void) {
  int32_t rem = 5;

  // Zero ratio_spr guard
  TEST_ASSERT_EQUAL_INT32(0, Position_ConvertStepDeltaToCounts(10, 0, 4, &rem));
  TEST_ASSERT_EQUAL_INT32(5, rem);

  // NULL remainder pointer safe execution
  TEST_ASSERT_EQUAL_INT32(40, Position_ConvertStepDeltaToCounts(10, 1, 4, NULL));
}

/* --- Hardware Delta & Position Tracking Tests --- */

void test_step_delta_normal(void) {
  TEST_ASSERT_EQUAL_UINT16(0, Position_CalcStepDelta(500, 500));
  TEST_ASSERT_EQUAL_UINT16(5, Position_CalcStepDelta(105, 100));
  TEST_ASSERT_EQUAL_UINT16(500, Position_CalcStepDelta(1000, 500));
  TEST_ASSERT_EQUAL_UINT16(1, Position_CalcStepDelta(1, 0));
}

void test_step_delta_rollover_16bit(void) {
  // Direct 65535 (0xFFFF) -> 0 wrap: 1 step
  TEST_ASSERT_EQUAL_UINT16(1, Position_CalcStepDelta(0, 65535));

  // 65535 -> 5: (65535->0 = 1) + 5 = 6 steps
  TEST_ASSERT_EQUAL_UINT16(6, Position_CalcStepDelta(5, 65535));

  // 65500 -> 100: (65536 - 65500) + 100 = 136 steps
  TEST_ASSERT_EQUAL_UINT16(136, Position_CalcStepDelta(100, 65500));

  // 60000 -> 10000: (65536 - 60000) + 10000 = 15536 steps
  TEST_ASSERT_EQUAL_UINT16(15536, Position_CalcStepDelta(10000, 60000));
}

void test_accumulate_step_position_forward(void) {
  TEST_ASSERT_EQUAL_INT64(110, Position_AccumulateStepPosition(100, 10, false));
  TEST_ASSERT_EQUAL_INT64(-25, Position_AccumulateStepPosition(-50, 25, false));
  TEST_ASSERT_EQUAL_INT64(0, Position_AccumulateStepPosition(0, 0, false));
  TEST_ASSERT_EQUAL_INT64(1000000, Position_AccumulateStepPosition(999900, 100, false));
  // 64-bit coordinates exceeding 32-bit integer limits (> 2^31 - 1)
  TEST_ASSERT_EQUAL_INT64(5000000010LL, Position_AccumulateStepPosition(5000000000LL, 10, false));
  TEST_ASSERT_EQUAL_INT64(-4999999990LL, Position_AccumulateStepPosition(-5000000000LL, 10, false));
}

void test_accumulate_step_position_reverse(void) {
  TEST_ASSERT_EQUAL_INT64(90, Position_AccumulateStepPosition(100, 10, true));
  TEST_ASSERT_EQUAL_INT64(-75, Position_AccumulateStepPosition(-50, 25, true));
  TEST_ASSERT_EQUAL_INT64(0, Position_AccumulateStepPosition(0, 0, true));
  TEST_ASSERT_EQUAL_INT64(999800, Position_AccumulateStepPosition(999900, 100, true));
  // 64-bit coordinates exceeding 32-bit integer limits (< -2^31)
  TEST_ASSERT_EQUAL_INT64(4999999990LL, Position_AccumulateStepPosition(5000000000LL, 10, true));
  TEST_ASSERT_EQUAL_INT64(-5000000010LL, Position_AccumulateStepPosition(-5000000000LL, 10, true));
}

void test_realign_position_counters(void) {
  config.persistent.ratio_spr = 1;
  config.persistent.ratio_epr = 4;

  PositionCounters_t pos = {
      .encoder_pos = 9999,
      .commanded_pos = 9999,
      .step_rem = 12,
      .step_cnt_prev = 123,
      .step_dcnt = 456,
      .step_reverse = false
  };

  // Realign to encoder position 4000
  Position_RealignCounters(&pos, 4000, &config, 5000);
  TEST_ASSERT_EQUAL_INT64(4000, pos.encoder_pos);
  TEST_ASSERT_EQUAL_INT64(4000, pos.commanded_pos);
  TEST_ASSERT_EQUAL_INT32(0, pos.step_rem);
  TEST_ASSERT_EQUAL_UINT16(5000, pos.step_cnt_prev);
  TEST_ASSERT_EQUAL_UINT16(0, pos.step_dcnt);

  // Negative encoder position (-2000 counts)
  pos.step_rem = -5;
  Position_RealignCounters(&pos, -2000, &config, 1000);
  TEST_ASSERT_EQUAL_INT64(-2000, pos.encoder_pos);
  TEST_ASSERT_EQUAL_INT64(-2000, pos.commanded_pos);
  TEST_ASSERT_EQUAL_INT32(0, pos.step_rem);
  TEST_ASSERT_EQUAL_UINT16(1000, pos.step_cnt_prev);
  TEST_ASSERT_EQUAL_UINT16(0, pos.step_dcnt);

  // Realign to large 64-bit position
  Position_RealignCounters(&pos, 8000000000LL, &config, 2000);
  TEST_ASSERT_EQUAL_INT64(8000000000LL, pos.encoder_pos);
  TEST_ASSERT_EQUAL_INT64(8000000000LL, pos.commanded_pos);
  TEST_ASSERT_EQUAL_INT32(0, pos.step_rem);

  // Realign to large negative 64-bit position
  Position_RealignCounters(&pos, -12000000000LL, &config, 3000);
  TEST_ASSERT_EQUAL_INT64(-12000000000LL, pos.encoder_pos);
  TEST_ASSERT_EQUAL_INT64(-12000000000LL, pos.commanded_pos);
  TEST_ASSERT_EQUAL_INT32(0, pos.step_rem);

  // Null safety
  Position_RealignCounters(NULL, 1000, &config, 0);
}

void test_filter_step_blanking_normal_200khz(void) {
  uint32_t accum = 0;
  uint32_t out_period = 0;
  // 200 kHz = 240 ticks @ 48 MHz. Threshold = 168 ticks (3.5 us).
  for (int i = 0; i < 5; i++) {
    uint16_t step = Position_FilterStepWithBlanking(240, 168, &accum, &out_period);
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
  TEST_ASSERT_EQUAL_UINT16(1, Position_FilterStepWithBlanking(240, threshold, &accum, &out_period));
  TEST_ASSERT_EQUAL_UINT32(240, out_period);
  TEST_ASSERT_EQUAL_UINT32(0, accum);

  // Glitch 1: 30 ticks (0.625 us) later -> rejected!
  TEST_ASSERT_EQUAL_UINT16(0, Position_FilterStepWithBlanking(30, threshold, &accum, &out_period));
  TEST_ASSERT_EQUAL_UINT32(30, accum);

  // Glitch 2: 40 ticks (0.833 us) later -> rejected!
  TEST_ASSERT_EQUAL_UINT16(0, Position_FilterStepWithBlanking(40, threshold, &accum, &out_period));
  TEST_ASSERT_EQUAL_UINT32(70, accum);

  // Genuine step arriving 170 ticks after Glitch 2 (total 30 + 40 + 170 = 240 ticks from Step 1)
  TEST_ASSERT_EQUAL_UINT16(1, Position_FilterStepWithBlanking(170, threshold, &accum, &out_period));
  TEST_ASSERT_EQUAL_UINT32(240, out_period);
  TEST_ASSERT_EQUAL_UINT32(0, accum);
}

void test_filter_step_blanking_standstill_primed(void) {
  uint32_t threshold = 168;
  uint32_t accum = threshold; // Primed for standstill
  uint32_t out_period = 0;

  // First step from standstill should trigger immediately regardless of interval
  TEST_ASSERT_EQUAL_UINT16(1, Position_FilterStepWithBlanking(100, threshold, &accum, &out_period));
  TEST_ASSERT_EQUAL_UINT32(268, out_period);
  TEST_ASSERT_EQUAL_UINT32(0, accum);
}

void test_filter_step_blanking_null_safety(void) {
  uint32_t out_period = 0;
  TEST_ASSERT_EQUAL_UINT16(0, Position_FilterStepWithBlanking(240, 168, NULL, &out_period));
}

int main(void) {
  UNITY_BEGIN();
  /* Ratio conversions */
  RUN_TEST(test_step_delta_integer_ratio_4_to_1);
  RUN_TEST(test_step_delta_fractional_ratio_200_to_1024);
  RUN_TEST(test_step_delta_reversals_no_drift);
  RUN_TEST(test_step_delta_microstepping_ratio);
  RUN_TEST(test_step_delta_protection_and_null);

  /* Hardware step deltas & accumulation */
  RUN_TEST(test_step_delta_normal);
  RUN_TEST(test_step_delta_rollover_16bit);
  RUN_TEST(test_accumulate_step_position_forward);
  RUN_TEST(test_accumulate_step_position_reverse);
  RUN_TEST(test_realign_position_counters);

  /* Glitch blanking */
  RUN_TEST(test_filter_step_blanking_normal_200khz);
  RUN_TEST(test_filter_step_blanking_glitch_rejection_and_recovery);
  RUN_TEST(test_filter_step_blanking_standstill_primed);
  RUN_TEST(test_filter_step_blanking_null_safety);
  return UNITY_END();
}

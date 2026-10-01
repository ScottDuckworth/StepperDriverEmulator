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
  TEST_ASSERT_EQUAL_INT32(110, AccumulateStepPosition(100, 10, false));
  TEST_ASSERT_EQUAL_INT32(-25, AccumulateStepPosition(-50, 25, false));
  TEST_ASSERT_EQUAL_INT32(0, AccumulateStepPosition(0, 0, false));
  TEST_ASSERT_EQUAL_INT32(1000000, AccumulateStepPosition(999900, 100, false));
}

void test_accumulate_step_position_reverse(void) {
  TEST_ASSERT_EQUAL_INT32(90, AccumulateStepPosition(100, 10, true));
  TEST_ASSERT_EQUAL_INT32(-75, AccumulateStepPosition(-50, 25, true));
  TEST_ASSERT_EQUAL_INT32(0, AccumulateStepPosition(0, 0, true));
  TEST_ASSERT_EQUAL_INT32(999800, AccumulateStepPosition(999900, 100, true));
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
  TEST_ASSERT_EQUAL_INT32(4000, pos.encoder_pos);
  TEST_ASSERT_EQUAL_INT32(1000, pos.step_pos);
  TEST_ASSERT_EQUAL_UINT16(5000, pos.step_cnt_prev);
  TEST_ASSERT_EQUAL_UINT16(0, pos.step_dcnt);

  // Negative encoder position (-2000 counts = -500 steps)
  RealignPositionCounters(&pos, -2000, &config, 1000);
  TEST_ASSERT_EQUAL_INT32(-2000, pos.encoder_pos);
  TEST_ASSERT_EQUAL_INT32(-500, pos.step_pos);
  TEST_ASSERT_EQUAL_UINT16(1000, pos.step_cnt_prev);
  TEST_ASSERT_EQUAL_UINT16(0, pos.step_dcnt);

  // Null safety
  RealignPositionCounters(NULL, 1000, &config, 0);
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_step_delta_normal);
  RUN_TEST(test_step_delta_rollover_16bit);
  RUN_TEST(test_accumulate_step_position_forward);
  RUN_TEST(test_accumulate_step_position_reverse);
  RUN_TEST(test_realign_position_counters);
  return UNITY_END();
}

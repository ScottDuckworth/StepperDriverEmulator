#include "unity.h"
#include "motion_planner.h"
#include "emulator_config.h"

static EmulatorConfig_t config;

void setUp(void) {
  config = (EmulatorConfig_t) DEFAULT_EMULATOR_CONFIG;
  config.kp = 0.5f;
  config.kfree = 1.0f;
}

void tearDown(void) {}

void test_planner_nominal_tracking_forward(void) {
  MotionPlanRequest_t req = {
      .commanded_pos = 100,
      .planned_encoder_pos = 0,
      .load_tension = 0,
      .now = 1000,
      .last_step_time = 0,
      .step_period_cnt = 0,
      .step_reverse = false,
      .is_freewheeling = false,
      .stall_tripped = false,
      .chunk_size = 8
  };
  MotionPlanResult_t res;

  PlanMotionStep(&config, &req, &res);

  // Large error (100 >= chunk_size 8): full chunk forward
  TEST_ASSERT_EQUAL_INT(1, res.dir);
  TEST_ASSERT_EQUAL_UINT16(8, res.count_to_emit);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 50.0f, res.target_velocity); // kp = 0.5 * 100 = 50.0
  TEST_ASSERT_FALSE(res.is_freewheeling);
  TEST_ASSERT_FALSE(res.stall_tripped);
  TEST_ASSERT_FALSE(res.stall_trip_event);
}

void test_planner_nominal_tracking_small_error(void) {
  MotionPlanRequest_t req = {
      .commanded_pos = 105,
      .planned_encoder_pos = 100,
      .load_tension = 0,
      .now = 1000,
      .last_step_time = 0,
      .step_period_cnt = 0,
      .step_reverse = false,
      .is_freewheeling = false,
      .stall_tripped = false,
      .chunk_size = 8
  };
  MotionPlanResult_t res;

  PlanMotionStep(&config, &req, &res);

  // Error = 5 < chunk_size 8: emit exactly 5 steps
  TEST_ASSERT_EQUAL_INT(1, res.dir);
  TEST_ASSERT_EQUAL_UINT16(5, res.count_to_emit);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 2.5f, res.target_velocity); // 0.5 * 5 = 2.5
}

void test_planner_nominal_tracking_reverse(void) {
  MotionPlanRequest_t req = {
      .commanded_pos = -50,
      .planned_encoder_pos = 0,
      .load_tension = 0,
      .now = 1000,
      .last_step_time = 0,
      .step_period_cnt = 0,
      .step_reverse = false,
      .is_freewheeling = false,
      .stall_tripped = false,
      .chunk_size = 8
  };
  MotionPlanResult_t res;

  PlanMotionStep(&config, &req, &res);

  TEST_ASSERT_EQUAL_INT(-1, res.dir);
  TEST_ASSERT_EQUAL_UINT16(8, res.count_to_emit);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, -25.0f, res.target_velocity); // 0.5 * (-50) = -25.0
}

void test_planner_feedforward_rate(void) {
  // 48 MHz / 480 ticks = 100 kHz steps
  // 4000 epr / 1000 spr = 4.0 ratio -> 400 counts/ms input rate
  MotionPlanRequest_t req = {
      .commanded_pos = 0,
      .planned_encoder_pos = 0, // error = 0
      .load_tension = 0,
      .now = 100,
      .last_step_time = 90, // diff = 10 <= 50
      .step_period_cnt = 480,
      .step_reverse = false,
      .is_freewheeling = false,
      .stall_tripped = false,
      .chunk_size = 8
  };
  MotionPlanResult_t res;

  PlanMotionStep(&config, &req, &res);
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 400.0f, res.target_velocity);

  // Reverse direction feedforward
  req.step_reverse = true;
  PlanMotionStep(&config, &req, &res);
  TEST_ASSERT_FLOAT_WITHIN(0.1f, -400.0f, res.target_velocity);
}

void test_planner_torque_deficit_stall_and_slip(void) {
  // Commanded forward (+1), but opposing tension load_tension = -1500 exceeds motor T0 (1000)
  // Net torque = 1000 + (1 * -1500) = -500 < 0
  // Motor cannot advance; instead slips backward under load:
  // v_slip = (1500 - 1000) * 1.0 = 500 counts/sec -> 0.5 counts/ms in direction -1
  MotionPlanRequest_t req = {
      .commanded_pos = 500,
      .planned_encoder_pos = 0,
      .load_tension = -1500,
      .now = 1000,
      .last_step_time = 0,
      .step_period_cnt = 0,
      .step_reverse = false,
      .is_freewheeling = false,
      .stall_tripped = false,
      .chunk_size = 8
  };
  MotionPlanResult_t res;

  PlanMotionStep(&config, &req, &res);

  TEST_ASSERT_EQUAL_INT(-1, res.dir); // pulled backward
  TEST_ASSERT_EQUAL_UINT16(8, res.count_to_emit);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, -0.5f, res.target_velocity);
  TEST_ASSERT_FALSE(res.stall_tripped); // error 500 < threshold 4000
  TEST_ASSERT_FALSE(res.stall_trip_event);
}

void test_planner_stall_trip_trigger_event(void) {
  // When error reaches or exceeds stall_threshold (4000) under torque deficit:
  MotionPlanRequest_t req = {
      .commanded_pos = 4000,
      .planned_encoder_pos = 0, // error = 4000 >= stall_threshold 4000
      .load_tension = -1500,
      .now = 1000,
      .last_step_time = 0,
      .step_period_cnt = 0,
      .step_reverse = false,
      .is_freewheeling = false,
      .stall_tripped = false,
      .chunk_size = 8
  };
  MotionPlanResult_t res;

  PlanMotionStep(&config, &req, &res);

  TEST_ASSERT_TRUE(res.stall_trip_event); // One-shot trigger event fired
  TEST_ASSERT_TRUE(res.stall_tripped);
  TEST_ASSERT_TRUE(res.is_freewheeling);
}

void test_planner_freewheeling_under_tension(void) {
  // When disengaged or tripped into freewheeling:
  MotionPlanRequest_t req = {
      .commanded_pos = 0,
      .planned_encoder_pos = 0,
      .load_tension = 2000, // 2000 * kfree(1.0) = 2000 counts/sec -> 2.0 counts/ms
      .now = 1000,
      .last_step_time = 0,
      .step_period_cnt = 0,
      .step_reverse = false,
      .is_freewheeling = true,
      .stall_tripped = false,
      .chunk_size = 8
  };
  MotionPlanResult_t res;

  PlanMotionStep(&config, &req, &res);

  TEST_ASSERT_EQUAL_INT(1, res.dir);
  TEST_ASSERT_EQUAL_UINT16(8, res.count_to_emit);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 2.0f, res.target_velocity);
  TEST_ASSERT_TRUE(res.is_freewheeling);
}

void test_planner_freewheeling_zero_tension_stops(void) {
  MotionPlanRequest_t req = {
      .commanded_pos = 0,
      .planned_encoder_pos = 0,
      .load_tension = 0,
      .now = 1000,
      .last_step_time = 0,
      .step_period_cnt = 0,
      .step_reverse = false,
      .is_freewheeling = true,
      .stall_tripped = false,
      .chunk_size = 8
  };
  MotionPlanResult_t res;

  PlanMotionStep(&config, &req, &res);

  TEST_ASSERT_EQUAL_INT(0, res.dir);
  TEST_ASSERT_EQUAL_UINT16(0, res.count_to_emit);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, res.target_velocity);
}

void test_planner_at_target_zero_error_stable(void) {
  // At target, load tension within holding torque (500 <= 1000)
  MotionPlanRequest_t req = {
      .commanded_pos = 1000,
      .planned_encoder_pos = 1000,
      .load_tension = 500,
      .now = 1000,
      .last_step_time = 0,
      .step_period_cnt = 0,
      .step_reverse = false,
      .is_freewheeling = false,
      .stall_tripped = false,
      .chunk_size = 8
  };
  MotionPlanResult_t res;

  PlanMotionStep(&config, &req, &res);

  TEST_ASSERT_EQUAL_INT(0, res.dir);
  TEST_ASSERT_EQUAL_UINT16(0, res.count_to_emit);
}

void test_planner_null_safety(void) {
  MotionPlanRequest_t req = {0};
  MotionPlanResult_t res = {0};

  // Safe against NULL
  PlanMotionStep(NULL, NULL, NULL);
  PlanMotionStep(&config, &req, NULL);
  PlanMotionStep(&config, NULL, &res);
  PlanMotionStep(NULL, &req, &res);
  TEST_ASSERT_EQUAL_INT(0, res.dir);
}

void test_planner_default_config_gains(void) {
  EmulatorConfig_t def_cfg = (EmulatorConfig_t) DEFAULT_EMULATOR_CONFIG;
  MotionPlanRequest_t req = {
      .commanded_pos = 100,
      .planned_encoder_pos = 0,
      .load_tension = 0,
      .now = 1000,
      .last_step_time = 0,
      .step_period_cnt = 0,
      .step_reverse = false,
      .is_freewheeling = false,
      .stall_tripped = false,
      .chunk_size = 8
  };
  MotionPlanResult_t res;

  PlanMotionStep(&def_cfg, &req, &res);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 10.0f, res.target_velocity); // default kp = 0.1 * 100 = 10.0
}

void test_planner_continuous_streaming_zero_error(void) {
  // Input rate = 400.0 counts/ms, error = 0
  // Pacing velocity matches input rate, but count_to_emit = 0 so motor does not overshoot
  MotionPlanRequest_t req = {
      .commanded_pos = 100,
      .planned_encoder_pos = 100, // error = 0
      .load_tension = 0,
      .now = 100,
      .last_step_time = 90,
      .step_period_cnt = 480, // 400 counts/ms
      .step_reverse = false,
      .is_freewheeling = false,
      .stall_tripped = false,
      .chunk_size = 8
  };
  MotionPlanResult_t res;

  PlanMotionStep(&config, &req, &res);
  TEST_ASSERT_EQUAL_UINT16(0, res.count_to_emit);
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 400.0f, res.target_velocity);
}

void test_planner_soft_knee_error_attenuation(void) {
  // epr = 4000, spr = 1000 -> 1 step = 4 counts.
  // Within nominal 1-step feedforward window (error = 4): eff_error = 0.0 (pure feedforward 400.0 counts/ms).
  MotionPlanRequest_t req = {
      .commanded_pos = 104,
      .planned_encoder_pos = 100, // error = 4 (nominal 1-step streaming)
      .load_tension = 0,
      .now = 100,
      .last_step_time = 90,
      .step_period_cnt = 480, // 400 counts/ms
      .step_reverse = false,
      .is_freewheeling = false,
      .stall_tripped = false,
      .chunk_size = 8
  };
  MotionPlanResult_t res;

  PlanMotionStep(&config, &req, &res);
  TEST_ASSERT_EQUAL_INT(1, res.dir);
  TEST_ASSERT_EQUAL_UINT16(4, res.count_to_emit);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 400.0f, res.target_velocity); // 0 phase modulation at nominal 1 step

  // When error = 6 (excess lag = 6 - 4 = 2 counts <= 4): eff_error = (2 * 2) / 4.0 = 1.0.
  // kp = 0.5 -> kp * eff_error = 0.5.
  // target_velocity = 400.0 + 0.5 = 400.5
  req.commanded_pos = 106;
  PlanMotionStep(&config, &req, &res);
  TEST_ASSERT_EQUAL_INT(1, res.dir);
  TEST_ASSERT_EQUAL_UINT16(6, res.count_to_emit);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 400.5f, res.target_velocity);

  // When error = 24 (excess lag = 24 - 4 = 20 counts > 4): eff_error = 20 (full linear gain).
  // kp * eff_error = 0.5 * 20 = 10.0.
  // target_velocity = 400.0 + 10.0 = 410.0
  req.commanded_pos = 124;
  PlanMotionStep(&config, &req, &res);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 410.0f, res.target_velocity);
}

void test_planner_large_64bit_coordinates(void) {
  MotionPlanRequest_t req = {
      .commanded_pos = 10000000005LL,
      .planned_encoder_pos = 10000000000LL, // error = 5 counts at 10 billion
      .load_tension = 0,
      .now = 1000,
      .last_step_time = 0,
      .step_period_cnt = 0,
      .step_reverse = false,
      .is_freewheeling = false,
      .stall_tripped = false,
      .chunk_size = 8
  };
  MotionPlanResult_t res;

  PlanMotionStep(&config, &req, &res);

  // Error = 5 < chunk_size 8: emit exactly 5 steps
  TEST_ASSERT_EQUAL_INT(1, res.dir);
  TEST_ASSERT_EQUAL_UINT16(5, res.count_to_emit);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 2.5f, res.target_velocity); // kp = 0.5 * 5 = 2.5
}

void test_planner_low_frequency_feedforward_50hz(void) {
  // 48 MHz / 960,000 ticks = 50 Hz steps
  // 4000 epr / 1000 spr = 4.0 ratio -> input_rate = 50 * 4 = 200 counts/sec = 0.2 counts/ms
  // With kff = 1.0, kp = 0.1, target_velocity remains 0.2 counts/ms across the 1-step window (takes 20 ms for 4 counts)
  config.kp = 0.1f;
  MotionPlanRequest_t req = {
      .commanded_pos = 104,
      .planned_encoder_pos = 100, // 4 counts error (1 step)
      .load_tension = 0,
      .now = 120,
      .last_step_time = 100, // 20 ms gap <= 50 ms timeout
      .step_period_cnt = 960000,
      .step_reverse = false,
      .is_freewheeling = false,
      .stall_tripped = false,
      .chunk_size = 4
  };
  MotionPlanResult_t res;

  PlanMotionStep(&config, &req, &res);
  TEST_ASSERT_EQUAL_INT(1, res.dir);
  TEST_ASSERT_EQUAL_UINT16(4, res.count_to_emit);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.2f, res.target_velocity);

  // When error reaches 0, no counts emitted to prevent overshoot
  req.commanded_pos = 100;
  PlanMotionStep(&config, &req, &res);
  TEST_ASSERT_EQUAL_UINT16(0, res.count_to_emit);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.2f, res.target_velocity);
}

void test_planner_low_frequency_timeout_extension(void) {
  // 20 Hz step rate: 48 MHz / 2,400,000 ticks = 50 ms period
  // dynamic timeout = 50 + 25 + 10 = 85 ms
  // input_rate = (48000 / 2400000) * 4.0 = 0.08 counts/ms
  config.kp = 0.1f;
  MotionPlanRequest_t req = {
      .commanded_pos = 104,
      .planned_encoder_pos = 100, // 4 counts error
      .load_tension = 0,
      .now = 170,
      .last_step_time = 100, // 70 ms gap: > 50 ms old limit, but <= 85 ms dynamic timeout
      .step_period_cnt = 2400000,
      .step_reverse = false,
      .is_freewheeling = false,
      .stall_tripped = false,
      .chunk_size = 4
  };
  MotionPlanResult_t res;

  PlanMotionStep(&config, &req, &res);
  // Pacing remains active across 70 ms gap with exactly 4 counts
  TEST_ASSERT_EQUAL_INT(1, res.dir);
  TEST_ASSERT_EQUAL_UINT16(4, res.count_to_emit);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.08f, res.target_velocity);

  // If time exceeds 85 ms dynamic timeout (e.g. 90 ms gap), input_rate drops to 0
  config.kp = 0.5f;
  req.now = 190;
  PlanMotionStep(&config, &req, &res);
  // With input_rate = 0, target_velocity is kp * 4 = 0.5 * 4 = 2.0 counts/ms
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 2.0f, res.target_velocity);
  TEST_ASSERT_EQUAL_UINT16(4, res.count_to_emit);

  // When error reaches 0 after timeout, 0 counts are emitted
  req.commanded_pos = 100;
  PlanMotionStep(&config, &req, &res);
  TEST_ASSERT_EQUAL_UINT16(0, res.count_to_emit);
}

void test_planner_standstill_single_step_kp_pacing(void) {
  // Single step from standstill: step_period_cnt = 0 (no frequency known)
  // error = 4 counts (1 step at 4000 epr / 1000 spr)
  // target_velocity must be kp * error
  EmulatorConfig_t test_cfg = (EmulatorConfig_t) DEFAULT_EMULATOR_CONFIG;
  test_cfg.kp = 0.05f; // User sets kp = 0.05
  test_cfg.kff = 1.0f;

  MotionPlanRequest_t req = {
      .commanded_pos = 4,
      .planned_encoder_pos = 0,
      .load_tension = 0,
      .now = 1000,
      .last_step_time = 1000,
      .step_period_cnt = 0,
      .step_reverse = false,
      .is_freewheeling = false,
      .stall_tripped = false,
      .chunk_size = 4
  };
  MotionPlanResult_t res;

  PlanMotionStep(&test_cfg, &req, &res);
  TEST_ASSERT_EQUAL_INT(1, res.dir);
  TEST_ASSERT_EQUAL_UINT16(4, res.count_to_emit);
  // Velocity is kp * 4 = 0.05 * 4 = 0.2 counts/ms (200 counts/s = 20 ms for 4 counts)
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.2f, res.target_velocity);

  // With kp = 0.1: velocity is 0.1 * 4 = 0.4 counts/ms (10 ms for 4 counts)
  test_cfg.kp = 0.1f;
  PlanMotionStep(&test_cfg, &req, &res);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.4f, res.target_velocity);
}

void test_planner_prevents_reversals_during_streaming(void) {
  // During forward streaming (input_rate = 0.08 counts/ms, 20 Hz):
  // Even if kp * error is negative and large (e.g. kp = 0.5, error = -4 -> -2.0 counts/ms),
  // target_velocity must NOT become negative and dir must remain positive (1).
  config.kp = 0.5f;
  config.kff = 1.0f;
  MotionPlanRequest_t req = {
      .commanded_pos = 100,
      .planned_encoder_pos = 104, // 4 counts ahead (error = -4)
      .load_tension = 0,
      .now = 120,
      .last_step_time = 100,
      .step_period_cnt = 2400000, // 20 Hz (input_rate = 0.08 counts/ms)
      .step_reverse = false,
      .is_freewheeling = false,
      .stall_tripped = false,
      .chunk_size = 4
  };
  MotionPlanResult_t res;

  PlanMotionStep(&config, &req, &res);
  // Must maintain forward direction (dir = 1), clamped velocity = 0.0, and 0 counts emitted
  TEST_ASSERT_EQUAL_INT(1, res.dir);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, res.target_velocity);
  TEST_ASSERT_EQUAL_UINT16(0, res.count_to_emit);

  // Similarly during reverse streaming:
  req.step_reverse = true;
  req.commanded_pos = 100;
  req.planned_encoder_pos = 96; // 4 counts ahead in reverse (error = +4)
  PlanMotionStep(&config, &req, &res);
  TEST_ASSERT_EQUAL_INT(-1, res.dir);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, res.target_velocity);
  TEST_ASSERT_EQUAL_UINT16(0, res.count_to_emit);
}

void test_planner_streaming_7khz(void) {
  EmulatorConfig_t test_cfg = (EmulatorConfig_t) DEFAULT_EMULATOR_CONFIG;
  // 7 kHz step rate: 48 MHz / 6857 ticks = 7000.14 Hz
  // input_rate = (48000 / 6857) * 4.0 = 28.0006 counts/ms
  MotionPlanRequest_t req = {
      .commanded_pos = 4,
      .planned_encoder_pos = 0,
      .load_tension = 0,
      .now = 100,
      .last_step_time = 100,
      .step_period_cnt = 6857,
      .step_reverse = false,
      .is_freewheeling = false,
      .stall_tripped = false,
      .chunk_size = 4
  };
  MotionPlanResult_t res;

  // Step 1 arrives
  PlanMotionStep(&test_cfg, &req, &res);
  TEST_ASSERT_EQUAL_INT(1, res.dir);
  TEST_ASSERT_EQUAL_UINT16(4, res.count_to_emit);
  // Velocity should be 28.0 counts/ms
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 28.0f, res.target_velocity);

  // Suppose chunk of 4 counts was emitted
  req.planned_encoder_pos = 4;
  // Next step arrives: commanded_pos = 8
  req.commanded_pos = 8;
  PlanMotionStep(&test_cfg, &req, &res);
  TEST_ASSERT_EQUAL_INT(1, res.dir);
  TEST_ASSERT_EQUAL_UINT16(4, res.count_to_emit);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 28.0f, res.target_velocity);
}

void test_planner_streaming_clamps_excessive_catchup_velocity(void) {
  EmulatorConfig_t test_cfg = (EmulatorConfig_t) DEFAULT_EMULATOR_CONFIG;
  // 7 kHz step rate: input_rate = 28.0 counts/ms
  // If an accumulated lag of 280 counts (70 steps) occurs:
  // Unclamped Kp * eff_error would add 28.0 counts/ms, resulting in 56.0 counts/ms (14 kHz 2x runaway).
  // Clamping must restrict catchup authority during active streaming to <= 1.25 * input_rate + kp * step_counts = 35.4 counts/ms.
  MotionPlanRequest_t req = {
      .commanded_pos = 300,
      .planned_encoder_pos = 20, // 280 counts of lag
      .load_tension = 0,
      .now = 100,
      .last_step_time = 100,
      .step_period_cnt = 6857,
      .step_reverse = false,
      .is_freewheeling = false,
      .stall_tripped = false,
      .chunk_size = 4
  };
  MotionPlanResult_t res;

  PlanMotionStep(&test_cfg, &req, &res);
  TEST_ASSERT_EQUAL_INT(1, res.dir);
  TEST_ASSERT_EQUAL_UINT16(4, res.count_to_emit);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 35.4f, res.target_velocity);
  TEST_ASSERT_TRUE(res.target_velocity < 40.0f); // Guaranteed well below 56.0 counts/ms runaway

  // Reverse streaming:
  req.step_reverse = true;
  req.commanded_pos = -300;
  req.planned_encoder_pos = -20; // -280 counts of lag in reverse
  PlanMotionStep(&test_cfg, &req, &res);
  TEST_ASSERT_EQUAL_INT(-1, res.dir);
  TEST_ASSERT_EQUAL_UINT16(4, res.count_to_emit);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, -35.4f, res.target_velocity);
  TEST_ASSERT_TRUE(res.target_velocity > -40.0f);
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_planner_nominal_tracking_forward);
  RUN_TEST(test_planner_nominal_tracking_small_error);
  RUN_TEST(test_planner_nominal_tracking_reverse);
  RUN_TEST(test_planner_feedforward_rate);
  RUN_TEST(test_planner_torque_deficit_stall_and_slip);
  RUN_TEST(test_planner_stall_trip_trigger_event);
  RUN_TEST(test_planner_freewheeling_under_tension);
  RUN_TEST(test_planner_freewheeling_zero_tension_stops);
  RUN_TEST(test_planner_at_target_zero_error_stable);
  RUN_TEST(test_planner_null_safety);
  RUN_TEST(test_planner_default_config_gains);
  RUN_TEST(test_planner_continuous_streaming_zero_error);
  RUN_TEST(test_planner_soft_knee_error_attenuation);
  RUN_TEST(test_planner_large_64bit_coordinates);
  RUN_TEST(test_planner_low_frequency_feedforward_50hz);
  RUN_TEST(test_planner_low_frequency_timeout_extension);
  RUN_TEST(test_planner_standstill_single_step_kp_pacing);
  RUN_TEST(test_planner_prevents_reversals_during_streaming);
  RUN_TEST(test_planner_streaming_7khz);
  RUN_TEST(test_planner_streaming_clamps_excessive_catchup_velocity);
  return UNITY_END();
}

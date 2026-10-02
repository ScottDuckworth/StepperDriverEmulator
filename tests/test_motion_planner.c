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
      .cfg = &config,
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

  PlanMotionStep(&req, &res);

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
      .cfg = &config,
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

  PlanMotionStep(&req, &res);

  // Error = 5 < chunk_size 8: emit exactly 5 steps
  TEST_ASSERT_EQUAL_INT(1, res.dir);
  TEST_ASSERT_EQUAL_UINT16(5, res.count_to_emit);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 2.5f, res.target_velocity); // 0.5 * 5 = 2.5
}

void test_planner_nominal_tracking_reverse(void) {
  MotionPlanRequest_t req = {
      .cfg = &config,
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

  PlanMotionStep(&req, &res);

  TEST_ASSERT_EQUAL_INT(-1, res.dir);
  TEST_ASSERT_EQUAL_UINT16(8, res.count_to_emit);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, -25.0f, res.target_velocity); // 0.5 * (-50) = -25.0
}

void test_planner_feedforward_rate(void) {
  // 48 MHz / 480 ticks = 100 kHz steps
  // 4000 epr / 1000 spr = 4.0 ratio -> 400 counts/ms input rate
  MotionPlanRequest_t req = {
      .cfg = &config,
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

  PlanMotionStep(&req, &res);
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 400.0f, res.target_velocity);

  // Reverse direction feedforward
  req.step_reverse = true;
  PlanMotionStep(&req, &res);
  TEST_ASSERT_FLOAT_WITHIN(0.1f, -400.0f, res.target_velocity);
}

void test_planner_torque_deficit_stall_and_slip(void) {
  // Commanded forward (+1), but opposing tension load_tension = -1500 exceeds motor T0 (1000)
  // Net torque = 1000 + (1 * -1500) = -500 < 0
  // Motor cannot advance; instead slips backward under load:
  // v_slip = (1500 - 1000) * 1.0 = 500 counts/sec -> 0.5 counts/ms in direction -1
  MotionPlanRequest_t req = {
      .cfg = &config,
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

  PlanMotionStep(&req, &res);

  TEST_ASSERT_EQUAL_INT(-1, res.dir); // pulled backward
  TEST_ASSERT_EQUAL_UINT16(8, res.count_to_emit);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, -0.5f, res.target_velocity);
  TEST_ASSERT_FALSE(res.stall_tripped); // error 500 < threshold 4000
  TEST_ASSERT_FALSE(res.stall_trip_event);
}

void test_planner_stall_trip_trigger_event(void) {
  // When error reaches or exceeds stall_threshold (4000) under torque deficit:
  MotionPlanRequest_t req = {
      .cfg = &config,
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

  PlanMotionStep(&req, &res);

  TEST_ASSERT_TRUE(res.stall_trip_event); // One-shot trigger event fired
  TEST_ASSERT_TRUE(res.stall_tripped);
  TEST_ASSERT_TRUE(res.is_freewheeling);
}

void test_planner_freewheeling_under_tension(void) {
  // When disengaged or tripped into freewheeling:
  MotionPlanRequest_t req = {
      .cfg = &config,
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

  PlanMotionStep(&req, &res);

  TEST_ASSERT_EQUAL_INT(1, res.dir);
  TEST_ASSERT_EQUAL_UINT16(8, res.count_to_emit);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 2.0f, res.target_velocity);
  TEST_ASSERT_TRUE(res.is_freewheeling);
}

void test_planner_freewheeling_zero_tension_stops(void) {
  MotionPlanRequest_t req = {
      .cfg = &config,
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

  PlanMotionStep(&req, &res);

  TEST_ASSERT_EQUAL_INT(0, res.dir);
  TEST_ASSERT_EQUAL_UINT16(0, res.count_to_emit);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, res.target_velocity);
}

void test_planner_at_target_zero_error_stable(void) {
  // At target, load tension within holding torque (500 <= 1000)
  MotionPlanRequest_t req = {
      .cfg = &config,
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

  PlanMotionStep(&req, &res);

  TEST_ASSERT_EQUAL_INT(0, res.dir);
  TEST_ASSERT_EQUAL_UINT16(0, res.count_to_emit);
}

void test_planner_null_safety(void) {
  MotionPlanRequest_t req = {0};
  MotionPlanResult_t res = {0};

  // Safe against NULL
  PlanMotionStep(NULL, NULL);
  PlanMotionStep(&req, NULL);
  PlanMotionStep(NULL, &res);
  PlanMotionStep(&req, &res); // req.cfg == NULL
  TEST_ASSERT_EQUAL_INT(0, res.dir);
}

void test_planner_default_config_gains(void) {
  EmulatorConfig_t def_cfg = (EmulatorConfig_t) DEFAULT_EMULATOR_CONFIG;
  MotionPlanRequest_t req = {
      .cfg = &def_cfg,
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

  PlanMotionStep(&req, &res);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 10.0f, res.target_velocity); // default kp = 0.1 * 100 = 10.0
}

void test_planner_continuous_streaming_zero_error(void) {
  // Input rate = 400.0 counts/ms, error = 0
  MotionPlanRequest_t req = {
      .cfg = &config,
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

  PlanMotionStep(&req, &res);
  TEST_ASSERT_EQUAL_INT(1, res.dir);
  TEST_ASSERT_EQUAL_UINT16(8, res.count_to_emit);
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 400.0f, res.target_velocity);
}

void test_planner_soft_knee_error_attenuation(void) {
  // epr = 4000, spr = 1000 -> 1 step = 4 counts.
  // When error = 2 (sub-step error <= 4): eff_error = (2 * 2) / 4.0 = 1.0.
  // kp = 0.5 -> kp * eff_error = 0.5.
  // target_velocity = 400.0 + 0.5 = 400.5
  MotionPlanRequest_t req = {
      .cfg = &config,
      .commanded_pos = 102,
      .planned_encoder_pos = 100, // error = 2
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

  PlanMotionStep(&req, &res);
  TEST_ASSERT_EQUAL_INT(1, res.dir);
  TEST_ASSERT_EQUAL_UINT16(8, res.count_to_emit);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 400.5f, res.target_velocity);

  // When error = 20 (multi-step error > 4): eff_error = 20 (full gain).
  // kp * eff_error = 0.5 * 20 = 10.0.
  // target_velocity = 400.0 + 10.0 = 410.0
  req.commanded_pos = 120;
  PlanMotionStep(&req, &res);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 410.0f, res.target_velocity);
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
  return UNITY_END();
}

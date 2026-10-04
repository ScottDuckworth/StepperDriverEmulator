#include "unity.h"
#include "motion.h"
#include "emulator_config.h"
#include "config_store.h"

static EmulatorConfig_t config;

void setUp(void) {
  config = (EmulatorConfig_t) DEFAULT_EMULATOR_CONFIG;
  config.kp_q12 = (q12_t){ .raw = 2048 };   // kp = 0.5
  config.kfree_q12 = (q12_t){ .raw = 4096 }; // kfree = 1.0
  ConfigStore_RefreshCachedValues(&config);
}

void tearDown(void) {}

/* ========================================================================= */
/* --- Motor Torque-Speed Curve Physics Tests ------------------------------ */
/* ========================================================================= */

void test_torque_curve_standstill(void) {
  TEST_ASSERT_EQUAL_INT32(1000, Motion_CalcMotorTorque(&config, 0));
}

void test_torque_curve_below_knee(void) {
  TEST_ASSERT_EQUAL_INT32(1000, Motion_CalcMotorTorque(&config, 500));
}

void test_torque_curve_at_knee(void) {
  TEST_ASSERT_EQUAL_INT32(1000, Motion_CalcMotorTorque(&config, 1000));
}

void test_torque_curve_midpoint(void) {
  // Midpoint between v_knee (1000) and v_max (8000) is 4500
  // Torque should be midpoint between t0 (1000) and t_min (200), which is 600
  TEST_ASSERT_EQUAL_INT32(600, Motion_CalcMotorTorque(&config, 4500));
}

void test_torque_curve_at_max(void) {
  TEST_ASSERT_EQUAL_INT32(200, Motion_CalcMotorTorque(&config, 8000));
}

void test_torque_curve_above_max(void) {
  TEST_ASSERT_EQUAL_INT32(200, Motion_CalcMotorTorque(&config, 12000));
  TEST_ASSERT_EQUAL_INT32(200, Motion_CalcMotorTorque(&config, 50000));
}

void test_torque_curve_degenerate_vmax_less_than_knee(void) {
  config.torque_v_knee = 3000;
  config.torque_v_max = 2000; // Inverted / degenerate
  ConfigStore_RefreshCachedValues(&config);
  // Below knee: should still return t0
  TEST_ASSERT_EQUAL_INT32(1000, Motion_CalcMotorTorque(&config, 1000));
  // At or above knee: should return t_min without division by zero
  TEST_ASSERT_EQUAL_INT32(200, Motion_CalcMotorTorque(&config, 3500));
}

void test_torque_curve_null_config(void) {
  TEST_ASSERT_EQUAL_INT32(0, Motion_CalcMotorTorque(NULL, 1000));
}

/* ========================================================================= */
/* --- Net Torque Margin Tests --------------------------------------------- */
/* ========================================================================= */

void test_net_torque_no_load(void) {
  int32_t t_motor = 1000;
  // Forward motion under zero tension
  TEST_ASSERT_EQUAL_INT32(1000, Motion_CalcNetTorque(t_motor, 1, 0));
  // Reverse motion under zero tension
  TEST_ASSERT_EQUAL_INT32(1000, Motion_CalcNetTorque(t_motor, -1, 0));
}

void test_net_torque_aiding_load(void) {
  int32_t t_motor = 1000;
  // Forward motion with positive tension <= t_motor: margin is positive (+500)
  TEST_ASSERT_EQUAL_INT32(500, Motion_CalcNetTorque(t_motor, 1, 500));
  // Reverse motion with negative tension <= t_motor: margin is positive (+500)
  TEST_ASSERT_EQUAL_INT32(500, Motion_CalcNetTorque(t_motor, -1, -500));
  // Overrunning load exceeding t_motor: net margin is negative (-500)
  TEST_ASSERT_EQUAL_INT32(-500, Motion_CalcNetTorque(t_motor, 1, 1500));
  TEST_ASSERT_EQUAL_INT32(-500, Motion_CalcNetTorque(t_motor, -1, -1500));
}

void test_net_torque_opposing_sufficient(void) {
  int32_t t_motor = 1000;
  // Forward motion opposed by 600 tension -> net torque is positive (+400)
  TEST_ASSERT_EQUAL_INT32(400, Motion_CalcNetTorque(t_motor, 1, -600));
  // Reverse motion opposed by 600 tension -> net torque is positive (+400)
  TEST_ASSERT_EQUAL_INT32(400, Motion_CalcNetTorque(t_motor, -1, 600));
}

void test_net_torque_deficit(void) {
  int32_t t_motor = 1000;
  // Forward motion opposed by 1500 tension -> net torque is negative (-500)
  TEST_ASSERT_EQUAL_INT32(-500, Motion_CalcNetTorque(t_motor, 1, -1500));
  // Reverse motion opposed by 1500 tension -> net torque is negative (-500)
  TEST_ASSERT_EQUAL_INT32(-500, Motion_CalcNetTorque(t_motor, -1, 1500));
}

/* ========================================================================= */
/* --- Freewheeling & Dynamic Slip Velocity Tests -------------------------- */
/* ========================================================================= */

void test_freewheel_velocity_zero_tension(void) {
  TEST_ASSERT_EQUAL_INT32(0, Motion_CalcFreewheelVelocity(&config, 0));
}

void test_freewheel_velocity_proportional(void) {
  config.kfree_q12 = (q12_t){ .raw = 20 }; // 0.005 * 4096 = 20
  ConfigStore_RefreshCachedValues(&config);
  // 1000 * 0.005 = 5 counts/sec (with Q12: 1000 * 20 = 20000 >> 12 = 4 counts/sec)
  TEST_ASSERT_INT32_WITHIN(1, 5, Motion_CalcFreewheelVelocity(&config, 1000));
  TEST_ASSERT_INT32_WITHIN(1, -5, Motion_CalcFreewheelVelocity(&config, -1000));
}

void test_freewheel_velocity_clamping(void) {
  config.torque_v_max = 8000;
  config.kfree_q12 = (q12_t){ .raw = 20 };
  ConfigStore_RefreshCachedValues(&config);
  // Large tension would produce 50,000 counts/sec -> clamped to +8000
  TEST_ASSERT_EQUAL_INT32(8000, Motion_CalcFreewheelVelocity(&config, 10000000));
  TEST_ASSERT_EQUAL_INT32(-8000, Motion_CalcFreewheelVelocity(&config, -10000000));
}

void test_slip_velocity_holding_torque(void) {
  config.torque_t0 = 1000;
  config.kfree_q12 = (q12_t){ .raw = 20 };
  ConfigStore_RefreshCachedValues(&config);

  // Below holding torque shelf -> rotor does not slip (returns 0)
  TEST_ASSERT_EQUAL_INT32(0, Motion_CalcSlipVelocity(&config, 500, config.torque_t0));
  TEST_ASSERT_EQUAL_INT32(0, Motion_CalcSlipVelocity(&config, -500, config.torque_t0));
  TEST_ASSERT_EQUAL_INT32(0, Motion_CalcSlipVelocity(&config, 1000, config.torque_t0));
  TEST_ASSERT_EQUAL_INT32(0, Motion_CalcSlipVelocity(&config, -1000, config.torque_t0));
}

void test_slip_velocity_exceeding_holding_torque(void) {
  config.torque_t0 = 1000;
  config.torque_v_max = 8000;
  config.kfree_q12 = (q12_t){ .raw = 20 };
  ConfigStore_RefreshCachedValues(&config);

  // Tension 3000 exceeds t0 (1000) by 2000 -> slip = 2000 * 0.005 = 10 counts/sec
  TEST_ASSERT_INT32_WITHIN(1, 10, Motion_CalcSlipVelocity(&config, 3000, config.torque_t0));
  TEST_ASSERT_INT32_WITHIN(1, 10, Motion_CalcSlipVelocity(&config, -3000, config.torque_t0));

  // Huge tension -> clamped to v_max (8000)
  TEST_ASSERT_EQUAL_INT32(8000, Motion_CalcSlipVelocity(&config, 5000000, config.torque_t0));
}

void test_slip_velocity_at_dynamic_torque(void) {
  config.kfree_q12 = (q12_t){ .raw = 20 };
  config.torque_v_max = 8000;
  ConfigStore_RefreshCachedValues(&config);

  // At high speed where motor torque derates to 400:
  // Tension 300 <= t_motor 400 -> no slip
  TEST_ASSERT_EQUAL_INT32(0, Motion_CalcSlipVelocity(&config, 300, 400));
  TEST_ASSERT_EQUAL_INT32(0, Motion_CalcSlipVelocity(&config, -300, 400));

  // Tension 1400 exceeds t_motor 400 by 1000 -> slip = 1000 * 0.005 = 5 counts/sec
  TEST_ASSERT_INT32_WITHIN(1, 5, Motion_CalcSlipVelocity(&config, 1400, 400));
  TEST_ASSERT_INT32_WITHIN(1, 5, Motion_CalcSlipVelocity(&config, -1400, 400));
}

/* ========================================================================= */
/* --- Step Planning & Motion Execution Tests ------------------------------ */
/* ========================================================================= */

void test_planner_nominal_tracking_forward(void) {
  Motion_PlanStepRequest_t req = {
      .commanded_pos = 100,
      .planned_encoder_pos = 0,
      .load_tension = 0,
      .now = 1000,
      .last_step_time = 0,
      .step_period_cnt = 0,
      .step_reverse = false,
      .is_freewheeling = false,
      .stall_tripped = false,
      .chunk_size = 16
  };
  Motion_PlanStepResult_t res;

  Motion_PlanStep(&config, &req, &res);

  // Large error (100 >= chunk_size 16): full chunk forward
  TEST_ASSERT_EQUAL_INT(1, res.dir);
  TEST_ASSERT_EQUAL_UINT16(16, res.count_to_emit);
  TEST_ASSERT_EQUAL_INT32(50000, res.target_velocity); // kp = 0.5 * 1000 * 100 = 50,000 counts/sec
  TEST_ASSERT_FALSE(res.is_freewheeling);
  TEST_ASSERT_FALSE(res.stall_tripped);
  TEST_ASSERT_FALSE(res.stall_trip_event);
}

void test_planner_nominal_tracking_small_error(void) {
  Motion_PlanStepRequest_t req = {
      .commanded_pos = 105,
      .planned_encoder_pos = 100,
      .load_tension = 0,
      .now = 1000,
      .last_step_time = 0,
      .step_period_cnt = 0,
      .step_reverse = false,
      .is_freewheeling = false,
      .stall_tripped = false,
      .chunk_size = 16
  };
  Motion_PlanStepResult_t res;

  Motion_PlanStep(&config, &req, &res);

  // Error = 5 < chunk_size 16: emit exactly 5 steps
  TEST_ASSERT_EQUAL_INT(1, res.dir);
  TEST_ASSERT_EQUAL_UINT16(5, res.count_to_emit);
  TEST_ASSERT_EQUAL_INT32(2500, res.target_velocity); // 0.5 * 1000 * 5 = 2,500 counts/sec
}

void test_planner_nominal_tracking_reverse(void) {
  Motion_PlanStepRequest_t req = {
      .commanded_pos = -50,
      .planned_encoder_pos = 0,
      .load_tension = 0,
      .now = 1000,
      .last_step_time = 0,
      .step_period_cnt = 0,
      .step_reverse = false,
      .is_freewheeling = false,
      .stall_tripped = false,
      .chunk_size = 16
  };
  Motion_PlanStepResult_t res;

  Motion_PlanStep(&config, &req, &res);

  TEST_ASSERT_EQUAL_INT(-1, res.dir);
  TEST_ASSERT_EQUAL_UINT16(16, res.count_to_emit);
  TEST_ASSERT_EQUAL_INT32(-25000, res.target_velocity); // 0.5 * 1000 * (-50) = -25,000 counts/sec
}

void test_planner_feedforward_rate(void) {
  // 48 MHz / 480 ticks = 100 kHz steps
  // 4000 epr / 1000 spr = 4.0 ratio -> 400,000 counts/sec input rate
  Motion_PlanStepRequest_t req = {
      .commanded_pos = 0,
      .planned_encoder_pos = 0, // error = 0
      .load_tension = 0,
      .now = 100,
      .last_step_time = 90, // diff = 10 <= 50
      .step_period_cnt = 480,
      .step_reverse = false,
      .is_freewheeling = false,
      .stall_tripped = false,
      .chunk_size = 16
  };
  Motion_PlanStepResult_t res;

  Motion_PlanStep(&config, &req, &res);
  TEST_ASSERT_EQUAL_INT32(400000, res.target_velocity);

  // Reverse direction feedforward
  req.step_reverse = true;
  Motion_PlanStep(&config, &req, &res);
  TEST_ASSERT_EQUAL_INT32(-400000, res.target_velocity);
}

void test_planner_torque_deficit_stall_and_slip(void) {
  /* Commanded forward (+1) at low speed within knee:
   * error = 2 -> target_velocity = 1000 counts/s <= v_knee 1000 -> T(v) = T0 = 1000
   * Opposing tension load_tension = -1500 exceeds motor T0 (1000).
   * Motor cannot advance; instead slips backward under load:
   * v_slip = (1500 - 1000) * 1.0 = 500 counts/sec in direction -1 */
  Motion_PlanStepRequest_t req = {
      .commanded_pos = 2,
      .planned_encoder_pos = 0,
      .load_tension = -1500,
      .now = 1000,
      .last_step_time = 0,
      .step_period_cnt = 0,
      .step_reverse = false,
      .is_freewheeling = false,
      .stall_tripped = false,
      .chunk_size = 16
  };
  Motion_PlanStepResult_t res;

  Motion_PlanStep(&config, &req, &res);

  TEST_ASSERT_EQUAL_INT(-1, res.dir); /* pulled backward */
  TEST_ASSERT_EQUAL_UINT16(16, res.count_to_emit);
  TEST_ASSERT_EQUAL_INT32(-500, res.target_velocity);
  TEST_ASSERT_FALSE(res.stall_tripped); /* error 2 < threshold 4000 */
  TEST_ASSERT_FALSE(res.stall_trip_event);

  /* At high commanded velocity (error = 500 -> 250,000 counts/s >= v_max 8000):
   * Motor pull-out torque derates to t_min = 200.
   * Dynamic slip velocity: v_slip = (1500 - 200) * 1.0 = 1300 counts/s in direction -1 */
  req.commanded_pos = 500;
  Motion_PlanStep(&config, &req, &res);
  TEST_ASSERT_EQUAL_INT(-1, res.dir);
  TEST_ASSERT_EQUAL_INT32(-1300, res.target_velocity);
}

void test_planner_stall_trip_trigger_event(void) {
  // When error reaches or exceeds stall_threshold (4000) under torque deficit:
  Motion_PlanStepRequest_t req = {
      .commanded_pos = 4000,
      .planned_encoder_pos = 0, // error = 4000 >= stall_threshold 4000
      .load_tension = -1500,
      .now = 1000,
      .last_step_time = 0,
      .step_period_cnt = 0,
      .step_reverse = false,
      .is_freewheeling = false,
      .stall_tripped = false,
      .chunk_size = 16
  };
  Motion_PlanStepResult_t res;

  Motion_PlanStep(&config, &req, &res);

  TEST_ASSERT_TRUE(res.stall_trip_event); // One-shot trigger event fired
  TEST_ASSERT_TRUE(res.stall_tripped);
  TEST_ASSERT_TRUE(res.is_freewheeling);
}

void test_planner_overrunning_aiding_tension_slip_and_stall(void) {
  /* Commanded forward (+1) at low speed within knee:
   * error = 2 -> target_velocity = 1000 counts/s <= v_knee 1000 -> T(v) = T0 = 1000
   * Aiding tension load_tension = +1500 exceeds motor T0 (1000).
   * Motor slips forward in direction of tension:
   * v_slip = (1500 - 1000) * 1.0 = 500 counts/s in direction +1 */
  Motion_PlanStepRequest_t req = {
      .commanded_pos = 2,
      .planned_encoder_pos = 0,
      .load_tension = 1500,
      .now = 1000,
      .last_step_time = 0,
      .step_period_cnt = 0,
      .step_reverse = false,
      .is_freewheeling = false,
      .stall_tripped = false,
      .chunk_size = 16
  };
  Motion_PlanStepResult_t res;
  Motion_PlanStep(&config, &req, &res);

  TEST_ASSERT_EQUAL_INT(1, res.dir); /* pulled forward in direction of tension */
  TEST_ASSERT_EQUAL_UINT16(16, res.count_to_emit);
  TEST_ASSERT_EQUAL_INT32(500, res.target_velocity);
  TEST_ASSERT_FALSE(res.stall_tripped); /* error 2 < threshold 4000 */
  TEST_ASSERT_FALSE(res.stall_trip_event);

  /* At high speed (error = 500 -> 250,000 counts/s >= v_max 8000):
   * Motor pull-out torque derates to t_min = 200.
   * Dynamic slip velocity: v_slip = (1500 - 200) * 1.0 = 1300 counts/s in direction +1 */
  req.commanded_pos = 500;
  Motion_PlanStep(&config, &req, &res);
  TEST_ASSERT_EQUAL_INT(1, res.dir);
  TEST_ASSERT_EQUAL_INT32(1300, res.target_velocity);

  /* Stall trip event when error reaches or exceeds stall_threshold 4000 */
  req.commanded_pos = 4000;
  Motion_PlanStep(&config, &req, &res);
  TEST_ASSERT_TRUE(res.stall_trip_event);
  TEST_ASSERT_TRUE(res.stall_tripped);
  TEST_ASSERT_TRUE(res.is_freewheeling);
}

void test_planner_freewheeling_under_tension(void) {
  // When disengaged or tripped into freewheeling:
  Motion_PlanStepRequest_t req = {
      .commanded_pos = 0,
      .planned_encoder_pos = 0,
      .load_tension = 2000, // 2000 * kfree(1.0) = 2000 counts/sec
      .now = 1000,
      .last_step_time = 0,
      .step_period_cnt = 0,
      .step_reverse = false,
      .is_freewheeling = true,
      .stall_tripped = false,
      .chunk_size = 16
  };
  Motion_PlanStepResult_t res;

  Motion_PlanStep(&config, &req, &res);

  TEST_ASSERT_EQUAL_INT(1, res.dir);
  TEST_ASSERT_EQUAL_UINT16(16, res.count_to_emit);
  TEST_ASSERT_EQUAL_INT32(2000, res.target_velocity);
  TEST_ASSERT_TRUE(res.is_freewheeling);
}

void test_planner_freewheeling_zero_tension_stops(void) {
  Motion_PlanStepRequest_t req = {
      .commanded_pos = 0,
      .planned_encoder_pos = 0,
      .load_tension = 0,
      .now = 1000,
      .last_step_time = 0,
      .step_period_cnt = 0,
      .step_reverse = false,
      .is_freewheeling = true,
      .stall_tripped = false,
      .chunk_size = 16
  };
  Motion_PlanStepResult_t res;

  Motion_PlanStep(&config, &req, &res);

  TEST_ASSERT_EQUAL_INT(0, res.dir);
  TEST_ASSERT_EQUAL_UINT16(0, res.count_to_emit);
  TEST_ASSERT_EQUAL_INT32(0, res.target_velocity);
}

void test_planner_at_target_zero_error_stable(void) {
  // At target, load tension within holding torque (500 <= 1000)
  Motion_PlanStepRequest_t req = {
      .commanded_pos = 1000,
      .planned_encoder_pos = 1000,
      .load_tension = 500,
      .now = 1000,
      .last_step_time = 0,
      .step_period_cnt = 0,
      .step_reverse = false,
      .is_freewheeling = false,
      .stall_tripped = false,
      .chunk_size = 16
  };
  Motion_PlanStepResult_t res;

  Motion_PlanStep(&config, &req, &res);

  TEST_ASSERT_EQUAL_INT(0, res.dir);
  TEST_ASSERT_EQUAL_UINT16(0, res.count_to_emit);
}

void test_planner_at_target_zero_error_tension_exceeds_holding_torque(void) {
  /* At target (error = 0, input_rate = 0), external load tension 1500 exceeds holding torque t0 (1000).
   * Motor slips in the direction of the tension (+1) at v_slip = (1500 - 1000) * 1.0 = 500 counts/sec */
  Motion_PlanStepRequest_t req = {
      .commanded_pos = 1000,
      .planned_encoder_pos = 1000,
      .load_tension = 1500,
      .now = 1000,
      .last_step_time = 0,
      .step_period_cnt = 0,
      .step_reverse = false,
      .is_freewheeling = false,
      .stall_tripped = false,
      .chunk_size = 16
  };
  Motion_PlanStepResult_t res;

  Motion_PlanStep(&config, &req, &res);

  TEST_ASSERT_EQUAL_INT(1, res.dir);
  TEST_ASSERT_EQUAL_UINT16(16, res.count_to_emit);
  TEST_ASSERT_EQUAL_INT32(500, res.target_velocity);

  /* Negative tension exceeding holding torque slips in direction -1 */
  req.load_tension = -1500;
  Motion_PlanStep(&config, &req, &res);
  TEST_ASSERT_EQUAL_INT(-1, res.dir);
  TEST_ASSERT_EQUAL_UINT16(16, res.count_to_emit);
  TEST_ASSERT_EQUAL_INT32(-500, res.target_velocity);
}

void test_planner_null_safety(void) {
  Motion_PlanStepRequest_t req = {0};
  Motion_PlanStepResult_t res = {0};

  // Safe against NULL
  Motion_PlanStep(NULL, NULL, NULL);
  Motion_PlanStep(&config, &req, NULL);
  Motion_PlanStep(&config, NULL, &res);
  Motion_PlanStep(NULL, &req, &res);
  TEST_ASSERT_EQUAL_INT(0, res.dir);
}

void test_planner_default_config_gains(void) {
  EmulatorConfig_t def_cfg = (EmulatorConfig_t) DEFAULT_EMULATOR_CONFIG;
  ConfigStore_RefreshCachedValues(&def_cfg);

  Motion_PlanStepRequest_t req = {
      .commanded_pos = 100,
      .planned_encoder_pos = 0,
      .load_tension = 0,
      .now = 1000,
      .last_step_time = 0,
      .step_period_cnt = 0,
      .step_reverse = false,
      .is_freewheeling = false,
      .stall_tripped = false,
      .chunk_size = 16
  };
  Motion_PlanStepResult_t res;

  Motion_PlanStep(&def_cfg, &req, &res);

  // Default Kp is 0.1 (410 in Q12), so error of 100 yields ~10,000 counts/sec (10009 counts/sec)
  TEST_ASSERT_INT32_WITHIN(10, 10000, res.target_velocity);
}

void test_planner_continuous_streaming_zero_error(void) {
  // Input rate = 400,000 counts/sec, error = 0
  // Pacing velocity matches input rate, but count_to_emit = 0 so motor does not overshoot
  Motion_PlanStepRequest_t req = {
      .commanded_pos = 100,
      .planned_encoder_pos = 100, // error = 0
      .load_tension = 0,
      .now = 100,
      .last_step_time = 90,
      .step_period_cnt = 480, // 400,000 counts/sec
      .step_reverse = false,
      .is_freewheeling = false,
      .stall_tripped = false,
      .chunk_size = 16
  };
  Motion_PlanStepResult_t res;

  Motion_PlanStep(&config, &req, &res);
  TEST_ASSERT_EQUAL_UINT16(0, res.count_to_emit);
  TEST_ASSERT_EQUAL_INT32(400000, res.target_velocity);
}

void test_planner_soft_knee_error_attenuation(void) {
  // During active streaming at 50 kHz (input_rate = 200,000 counts/sec):
  // Discrete step arrivals cause error to cycle between 0 and 1 step (4 counts).
  // 1. With error = 4 (exactly 1 step), effective error after subtracting feedforward window (4 counts) is 0.
  // Pacing remains exactly 200,000 counts/sec without cyclic frequency modulation!
  Motion_PlanStepRequest_t req = {
      .commanded_pos = 1004,
      .planned_encoder_pos = 1000, // error = 4 counts = 1 step
      .load_tension = 0,
      .now = 200,
      .last_step_time = 190,
      .step_period_cnt = 960, // 200,000 counts/sec
      .step_reverse = false,
      .is_freewheeling = false,
      .stall_tripped = false,
      .chunk_size = 16
  };
  Motion_PlanStepResult_t res;

  Motion_PlanStep(&config, &req, &res);
  TEST_ASSERT_EQUAL_INT32(200000, res.target_velocity);

  // 2. With error = 2 (halfway through the step), effective error is 0.
  req.commanded_pos = 1002;
  Motion_PlanStep(&config, &req, &res);
  TEST_ASSERT_EQUAL_INT32(200000, res.target_velocity);

  // 3. With error = 8 (2 steps behind), effective error is (8 - 4) = 4 counts.
  // Restoring term Kp * 4 = 0.5 * 1000 * 4 = 2,000 counts/sec accelerates motor to 202,000 counts/sec.
  req.commanded_pos = 1008;
  Motion_PlanStep(&config, &req, &res);
  TEST_ASSERT_EQUAL_INT32(202000, res.target_velocity);

  // 4. Overshoot during forward streaming: error = -2.
  // Negative error is outside deadband, attenuates quadratically:
  // abs_err = 2 <= 4 -> eff_error = (-2 * 2) * inv_counts_per_step = -1 count.
  // Target velocity = 200,000 + 0.5 * 1000 * (-1) = 199,500 counts/sec.
  req.commanded_pos = 998;
  Motion_PlanStep(&config, &req, &res);
  TEST_ASSERT_EQUAL_INT32(199500, res.target_velocity);
}

void test_planner_large_64bit_coordinates(void) {
  // Test planning across large 64-bit coordinates exceeding 32-bit signed limits (> 2^31 - 1)
  int64_t base_coord = 5000000000LL;
  Motion_PlanStepRequest_t req = {
      .commanded_pos = base_coord + 20,
      .planned_encoder_pos = base_coord,
      .load_tension = 0,
      .now = 1000,
      .last_step_time = 0,
      .step_period_cnt = 0,
      .step_reverse = false,
      .is_freewheeling = false,
      .stall_tripped = false,
      .chunk_size = 16
  };
  Motion_PlanStepResult_t res;

  Motion_PlanStep(&config, &req, &res);

  TEST_ASSERT_EQUAL_INT(1, res.dir);
  TEST_ASSERT_EQUAL_UINT16(16, res.count_to_emit);
  TEST_ASSERT_EQUAL_INT32(10000, res.target_velocity); // 0.5 * 1000 * 20 = 10,000 counts/sec
}

void test_planner_low_frequency_feedforward_50hz(void) {
  // 50 Hz step pulses: 48 MHz / 960000 ticks.
  // Period is 20 ms. Dynamic timeout is 20 + 10 + 10 = 40 ms.
  // 50 Hz * 4 counts/step = 200 counts/sec input rate.
  Motion_PlanStepRequest_t req = {
      .commanded_pos = 4,
      .planned_encoder_pos = 0,
      .load_tension = 0,
      .now = 125, // 25 ms since last step (exceeds default 20 ms interval)
      .last_step_time = 100,
      .step_period_cnt = 960000, // 50 Hz
      .step_reverse = false,
      .is_freewheeling = false,
      .stall_tripped = false,
      .chunk_size = 16
  };
  Motion_PlanStepResult_t res;

  Motion_PlanStep(&config, &req, &res);

  // Time diff = 25 ms <= dynamic step_timeout_ms (40 ms), so input_rate feedforward is active!
  // At error = 4 counts, feedforward window absorbs it, so velocity is exactly 200 counts/sec.
  TEST_ASSERT_EQUAL_INT(1, res.dir);
  TEST_ASSERT_EQUAL_UINT16(4, res.count_to_emit);
  TEST_ASSERT_EQUAL_INT32(200, res.target_velocity);
}

void test_planner_low_frequency_timeout_extension(void) {
  // At 20 Hz (period = 50 ms):
  // dynamic timeout is 50 + 25 + 10 = 85 ms.
  Motion_PlanStepRequest_t req = {
      .commanded_pos = 104,
      .planned_encoder_pos = 100,
      .load_tension = 0,
      .now = 170, // 70 ms since last pulse
      .last_step_time = 100,
      .step_period_cnt = 2400000, // 20 Hz
      .step_reverse = false,
      .is_freewheeling = false,
      .stall_tripped = false,
      .chunk_size = 16
  };
  Motion_PlanStepResult_t res;

  Motion_PlanStep(&config, &req, &res);

  // diff = 70 ms <= timeout (85 ms), feedforward remains active (20 Hz * 4 = 80 counts/sec)
  TEST_ASSERT_EQUAL_INT32(80, res.target_velocity);

  // If time exceeds 85 ms (e.g. 90 ms): steps have stopped!
  req.now = 190;
  Motion_PlanStep(&config, &req, &res);
  // diff = 90 > 85 ms -> feedforward drops to 0, motor paces purely by position error Kp * 4 = 0.5 * 1000 * 4 = 2,000 counts/sec
  TEST_ASSERT_EQUAL_INT32(2000, res.target_velocity);
  TEST_ASSERT_EQUAL_UINT16(4, res.count_to_emit);

  // When error reaches 0 after timeout, 0 counts are emitted
  req.commanded_pos = 100;
  Motion_PlanStep(&config, &req, &res);
  TEST_ASSERT_EQUAL_UINT16(0, res.count_to_emit);
}

void test_planner_standstill_single_step_kp_pacing(void) {
  // Single step from standstill: step_period_cnt = 0 (no frequency known)
  // error = 4 counts (1 step at 4000 epr / 1000 spr)
  EmulatorConfig_t test_cfg = (EmulatorConfig_t) DEFAULT_EMULATOR_CONFIG;
  test_cfg.kp_q12 = (q12_t){ .raw = 205 }; // kp = 0.05 (0.05 * 4096 = 205)
  test_cfg.kff_q12 = (q12_t){ .raw = 4096 };
  ConfigStore_RefreshCachedValues(&test_cfg);

  Motion_PlanStepRequest_t req = {
      .commanded_pos = 4,
      .planned_encoder_pos = 0,
      .load_tension = 0,
      .now = 1000,
      .last_step_time = 1000,
      .step_period_cnt = 0,
      .step_reverse = false,
      .is_freewheeling = false,
      .stall_tripped = false,
      .chunk_size = 16
  };
  Motion_PlanStepResult_t res;

  Motion_PlanStep(&test_cfg, &req, &res);
  TEST_ASSERT_EQUAL_INT(1, res.dir);
  TEST_ASSERT_EQUAL_UINT16(4, res.count_to_emit);
  // Velocity is kp * 4 = 0.05 * 1000 * 4 = 200 counts/sec
  TEST_ASSERT_EQUAL_INT32(200, res.target_velocity);

  // With kp = 0.1: velocity is 0.1 * 1000 * 4 = 400 counts/sec
  test_cfg.kp_q12 = (q12_t){ .raw = 410 };
  ConfigStore_RefreshCachedValues(&test_cfg);
  Motion_PlanStep(&test_cfg, &req, &res);
  TEST_ASSERT_EQUAL_INT32(400, res.target_velocity);
}

void test_planner_prevents_reversals_during_streaming(void) {
  // During forward streaming (input_rate = 80 counts/sec, 20 Hz):
  // Even if kp * error is negative and large (e.g. kp = 0.5, error = -4 -> -2,000 counts/sec),
  // target_velocity must NOT become negative and dir must remain positive (1).
  config.kp_q12 = (q12_t){ .raw = 2048 };
  config.kff_q12 = (q12_t){ .raw = 4096 };
  ConfigStore_RefreshCachedValues(&config);

  Motion_PlanStepRequest_t req = {
      .commanded_pos = 100,
      .planned_encoder_pos = 104, // 4 counts ahead (error = -4)
      .load_tension = 0,
      .now = 120,
      .last_step_time = 100,
      .step_period_cnt = 2400000, // 20 Hz (input_rate = 80 counts/sec)
      .step_reverse = false,
      .is_freewheeling = false,
      .stall_tripped = false,
      .chunk_size = 16
  };
  Motion_PlanStepResult_t res;

  Motion_PlanStep(&config, &req, &res);
  // Must maintain forward direction (dir = 1), clamped velocity = 0, and 0 counts emitted
  TEST_ASSERT_EQUAL_INT(1, res.dir);
  TEST_ASSERT_EQUAL_INT32(0, res.target_velocity);
  TEST_ASSERT_EQUAL_UINT16(0, res.count_to_emit);

  // Similarly during reverse streaming:
  req.step_reverse = true;
  req.commanded_pos = 100;
  req.planned_encoder_pos = 96; // 4 counts ahead in reverse (error = +4)
  Motion_PlanStep(&config, &req, &res);
  TEST_ASSERT_EQUAL_INT(-1, res.dir);
  TEST_ASSERT_EQUAL_INT32(0, res.target_velocity);
  TEST_ASSERT_EQUAL_UINT16(0, res.count_to_emit);
}

void test_planner_streaming_7khz(void) {
  EmulatorConfig_t test_cfg = (EmulatorConfig_t) DEFAULT_EMULATOR_CONFIG;
  ConfigStore_RefreshCachedValues(&test_cfg);

  // 7 kHz step rate: 48 MHz / 6857 ticks = 7000.14 Hz
  // input_rate = 7000 * 4 = 28,000 counts/sec
  Motion_PlanStepRequest_t req = {
      .commanded_pos = 4,
      .planned_encoder_pos = 0,
      .load_tension = 0,
      .now = 100,
      .last_step_time = 100,
      .step_period_cnt = 6857,
      .step_reverse = false,
      .is_freewheeling = false,
      .stall_tripped = false,
      .chunk_size = 16
  };
  Motion_PlanStepResult_t res;

  // Step 1 arrives
  Motion_PlanStep(&test_cfg, &req, &res);
  TEST_ASSERT_EQUAL_INT(1, res.dir);
  TEST_ASSERT_EQUAL_UINT16(4, res.count_to_emit);
  // Velocity should be ~28,000 counts/sec
  TEST_ASSERT_INT32_WITHIN(50, 28000, res.target_velocity);

  // Suppose chunk was emitted
  req.planned_encoder_pos = 4;
  // Next step arrives: commanded_pos = 8
  req.commanded_pos = 8;
  Motion_PlanStep(&test_cfg, &req, &res);
  TEST_ASSERT_EQUAL_INT(1, res.dir);
  TEST_ASSERT_EQUAL_UINT16(4, res.count_to_emit);
  TEST_ASSERT_INT32_WITHIN(50, 28000, res.target_velocity);
}

void test_planner_streaming_clamps_excessive_catchup_velocity(void) {
  EmulatorConfig_t test_cfg = (EmulatorConfig_t) DEFAULT_EMULATOR_CONFIG;
  ConfigStore_RefreshCachedValues(&test_cfg);

  // 7 kHz step rate: input_rate = 28,000 counts/sec
  // If an accumulated lag of 280 counts (70 steps) occurs:
  // Unclamped Kp * eff_error would add 28,000 counts/sec, resulting in 56,000 counts/sec (14 kHz 2x runaway).
  // Clamping must restrict catchup authority during active streaming to <= 1.25 * input_rate + kp * step_counts
  // = 28000 + 7000 + (0.1 * 1000 * 4) = 35,400 counts/sec.
  Motion_PlanStepRequest_t req = {
      .commanded_pos = 300,
      .planned_encoder_pos = 20, // 280 counts of lag
      .load_tension = 0,
      .now = 100,
      .last_step_time = 100,
      .step_period_cnt = 6857,
      .step_reverse = false,
      .is_freewheeling = false,
      .stall_tripped = false,
      .chunk_size = 16
  };
  Motion_PlanStepResult_t res;

  Motion_PlanStep(&test_cfg, &req, &res);
  TEST_ASSERT_EQUAL_INT(1, res.dir);
  TEST_ASSERT_EQUAL_UINT16(16, res.count_to_emit);
  TEST_ASSERT_INT32_WITHIN(100, 35400, res.target_velocity);
  TEST_ASSERT_TRUE(res.target_velocity < 40000); // Guaranteed well below 56,000 counts/sec runaway

  // Reverse streaming:
  req.step_reverse = true;
  req.commanded_pos = -300;
  req.planned_encoder_pos = -20; // -280 counts of lag in reverse
  Motion_PlanStep(&test_cfg, &req, &res);
  TEST_ASSERT_EQUAL_INT(-1, res.dir);
  TEST_ASSERT_EQUAL_UINT16(16, res.count_to_emit);
  TEST_ASSERT_INT32_WITHIN(100, -35400, res.target_velocity);
  TEST_ASSERT_TRUE(res.target_velocity > -40000);
}

void test_motion_calc_step_timeout_ms(void) {
  // High frequency / small period: default 50 ms
  TEST_ASSERT_EQUAL_UINT32(50, Motion_CalcStepTimeoutMs(0));
  TEST_ASSERT_EQUAL_UINT32(50, Motion_CalcStepTimeoutMs(240));
  TEST_ASSERT_EQUAL_UINT32(50, Motion_CalcStepTimeoutMs(47999));

  // 1 kHz (period 48000 ticks = 1 ms): dynamic timeout 1 + 0 + 10 = 11 <= 50 -> 50 ms
  TEST_ASSERT_EQUAL_UINT32(50, Motion_CalcStepTimeoutMs(48000));

  // 20 Hz (period 2400000 ticks = 50 ms): 50 + 25 + 10 = 85 ms
  TEST_ASSERT_EQUAL_UINT32(85, Motion_CalcStepTimeoutMs(2400000));

  // 10 Hz (period 4800000 ticks = 100 ms): 100 + 50 + 10 = 160 -> clamped to 150 ms
  TEST_ASSERT_EQUAL_UINT32(150, Motion_CalcStepTimeoutMs(4800000));
}

void test_motion_should_start(void) {
  EmulatorConfig_t test_cfg = (EmulatorConfig_t) DEFAULT_EMULATOR_CONFIG;
  test_cfg.torque_t0 = 1000;

  Motion_StartRequest_t req = {
      .commanded_pos = 100,
      .encoder_pos = 100,
      .load_tension = 0,
      .step_dcnt = 0,
      .now = 500,
      .last_step_time = 0,
      .step_reverse = false,
      .is_freewheeling = false,
      .stall_tripped = false
  };

  // Normal state, at rest, no load
  TEST_ASSERT_FALSE(Motion_ShouldStart(&test_cfg, &req));

  // Position error
  req.commanded_pos = 104;
  TEST_ASSERT_TRUE(Motion_ShouldStart(&test_cfg, &req));
  req.commanded_pos = 96;
  TEST_ASSERT_TRUE(Motion_ShouldStart(&test_cfg, &req));
  req.commanded_pos = 100;

  // New step arrived (dcnt != 0)
  req.step_dcnt = 1;
  TEST_ASSERT_TRUE(Motion_ShouldStart(&test_cfg, &req));
  req.step_dcnt = 0;

  // External load exceeding holding torque
  req.load_tension = 1200;
  TEST_ASSERT_TRUE(Motion_ShouldStart(&test_cfg, &req));
  req.load_tension = -1200;
  TEST_ASSERT_TRUE(Motion_ShouldStart(&test_cfg, &req));

  // Load within holding torque
  req.load_tension = 800;
  TEST_ASSERT_FALSE(Motion_ShouldStart(&test_cfg, &req));
  req.load_tension = -800;
  TEST_ASSERT_FALSE(Motion_ShouldStart(&test_cfg, &req));

  // Freewheeling mode
  req.is_freewheeling = true;
  req.load_tension = 0;
  TEST_ASSERT_FALSE(Motion_ShouldStart(&test_cfg, &req));
  req.load_tension = 50;
  TEST_ASSERT_TRUE(Motion_ShouldStart(&test_cfg, &req));

  // NULL safety
  TEST_ASSERT_FALSE(Motion_ShouldStart(NULL, &req));
  TEST_ASSERT_FALSE(Motion_ShouldStart(&test_cfg, NULL));
}

void test_motion_should_stop(void) {
  EmulatorConfig_t test_cfg = (EmulatorConfig_t) DEFAULT_EMULATOR_CONFIG;
  test_cfg.torque_t0 = 1000;

  Motion_StopRequest_t req = {
      .commanded_pos = 100,
      .encoder_pos = 100,
      .planned_encoder_pos = 100,
      .step_dcnt = 0,
      .load_tension = 0,
      .time_since_last_step_ms = 60,
      .step_timeout_ms = 50,
      .is_freewheeling = false
  };

  // 1. Nominal stop condition (converged, timed out, no load)
  TEST_ASSERT_TRUE(Motion_ShouldStop(&test_cfg, &req));

  // 2. Active step timeout not yet expired
  req.time_since_last_step_ms = 30;
  TEST_ASSERT_FALSE(Motion_ShouldStop(&test_cfg, &req));
  req.time_since_last_step_ms = 60;

  // 3. Encoder hasn't caught up to commanded pos
  req.encoder_pos = 96;
  TEST_ASSERT_FALSE(Motion_ShouldStop(&test_cfg, &req));
  req.encoder_pos = 100;

  // 4. Planned position has chunks ahead
  req.planned_encoder_pos = 108;
  TEST_ASSERT_FALSE(Motion_ShouldStop(&test_cfg, &req));
  req.planned_encoder_pos = 100;

  // 5. New input steps still present
  req.step_dcnt = 2;
  TEST_ASSERT_FALSE(Motion_ShouldStop(&test_cfg, &req));
  req.step_dcnt = 0;

  // 6. External tension exceeds holding torque
  req.load_tension = 1500;
  TEST_ASSERT_FALSE(Motion_ShouldStop(&test_cfg, &req));
  req.load_tension = 500;
  TEST_ASSERT_TRUE(Motion_ShouldStop(&test_cfg, &req));

  // 7. Freewheeling
  req.is_freewheeling = true;
  req.load_tension = 100;
  TEST_ASSERT_FALSE(Motion_ShouldStop(&test_cfg, &req));
  req.load_tension = 0;
  TEST_ASSERT_TRUE(Motion_ShouldStop(&test_cfg, &req));

  // 8. NULL safety
  TEST_ASSERT_FALSE(Motion_ShouldStop(NULL, &req));
  TEST_ASSERT_FALSE(Motion_ShouldStop(&test_cfg, NULL));
}

void test_motion_plan_and_emit_chunk(void) {
  uint32_t chunk_buf[16] = {0};
  uint8_t quad_state = 0;

  Motion_PlanStepRequest_t plan_req = {
      .commanded_pos = 100,
      .planned_encoder_pos = 0,
      .load_tension = 0,
      .now = 1000,
      .last_step_time = 0,
      .step_period_cnt = 0,
      .step_reverse = false,
      .is_freewheeling = false,
      .stall_tripped = false,
      .chunk_size = 16
  };
  Motion_ChunkRequest_t chunk_req = {
      .chunk = chunk_buf,
      .inout_quad_state = &quad_state
  };
  Motion_ChunkResult_t res = {0};

  bool ok = Motion_PlanAndEmitChunk(&config, &plan_req, &chunk_req, &res);
  TEST_ASSERT_TRUE(ok);
  TEST_ASSERT_EQUAL_INT8(16, res.delta);
  TEST_ASSERT_FALSE(res.stall_trip_event);
  TEST_ASSERT_GREATER_THAN_UINT16(0, res.arr);

  // Buffer entries should not all be 0 (valid GPIO BSRR bits set)
  TEST_ASSERT_NOT_EQUAL(0, chunk_buf[0]);
  TEST_ASSERT_NOT_EQUAL(0, chunk_buf[15]);

  // NULL safety
  TEST_ASSERT_FALSE(Motion_PlanAndEmitChunk(NULL, &plan_req, &chunk_req, &res));
  TEST_ASSERT_FALSE(Motion_PlanAndEmitChunk(&config, NULL, &chunk_req, &res));
  TEST_ASSERT_FALSE(Motion_PlanAndEmitChunk(&config, &plan_req, NULL, &res));
  Motion_ChunkRequest_t bad_chunk = chunk_req;
  bad_chunk.chunk = NULL;
  TEST_ASSERT_FALSE(Motion_PlanAndEmitChunk(&config, &plan_req, &bad_chunk, &res));
  bad_chunk = chunk_req;
  bad_chunk.inout_quad_state = NULL;
  TEST_ASSERT_FALSE(Motion_PlanAndEmitChunk(&config, &plan_req, &bad_chunk, &res));
}

void test_motion_prepare_start(void) {
  uint32_t quad_buf[32] = {0};
  uint8_t quad_state = 0;

  Motion_StartBuffers_t buf = {
      .chunk_size = 16,
      .quad_buffer = quad_buf,
      .inout_quad_state = &quad_state
  };
  Motion_StartResult_t res = {0};

  // 1. ShouldStart returns false (at target, idle)
  Motion_StartRequest_t idle_req = {
      .commanded_pos = 100,
      .encoder_pos = 100,
      .load_tension = 0,
      .step_dcnt = 0,
      .now = 500,
      .last_step_time = 0,
      .step_reverse = false,
      .is_freewheeling = false,
      .stall_tripped = false
  };
  TEST_ASSERT_FALSE(Motion_PrepareStart(&config, &idle_req, &buf, &res));

  // 2. Nominal start with position error
  Motion_StartRequest_t start_req = {
      .commanded_pos = 100,
      .encoder_pos = 0,
      .load_tension = 0,
      .step_dcnt = 0,
      .now = 500,
      .last_step_time = 0,
      .step_reverse = false,
      .is_freewheeling = false,
      .stall_tripped = false
  };
  bool started = Motion_PrepareStart(&config, &start_req, &buf, &res);
  TEST_ASSERT_TRUE(started);
  TEST_ASSERT_EQUAL_INT8(16, res.half_0_delta);
  TEST_ASSERT_EQUAL_INT8(16, res.half_1_delta);
  TEST_ASSERT_EQUAL_INT64(32, res.planned_encoder_pos);
  TEST_ASSERT_GREATER_THAN_UINT16(0, res.arr);
  TEST_ASSERT_FALSE(res.stall_trip_event);

  // Both halves of buffer should have valid patterns
  TEST_ASSERT_NOT_EQUAL(0, quad_buf[0]);
  TEST_ASSERT_NOT_EQUAL(0, quad_buf[16]);

  // 3. NULL safety
  TEST_ASSERT_FALSE(Motion_PrepareStart(NULL, &start_req, &buf, &res));
  TEST_ASSERT_FALSE(Motion_PrepareStart(&config, NULL, &buf, &res));
  TEST_ASSERT_FALSE(Motion_PrepareStart(&config, &start_req, NULL, &res));
  Motion_StartBuffers_t bad_buf = buf;
  bad_buf.quad_buffer = NULL;
  TEST_ASSERT_FALSE(Motion_PrepareStart(&config, &start_req, &bad_buf, &res));
  bad_buf = buf;
  bad_buf.inout_quad_state = NULL;
  TEST_ASSERT_FALSE(Motion_PrepareStart(&config, &start_req, &bad_buf, &res));
}

int main(void) {
  UNITY_BEGIN();

  /* Motor Torque-Speed Curve Physics Tests */
  RUN_TEST(test_torque_curve_standstill);
  RUN_TEST(test_torque_curve_below_knee);
  RUN_TEST(test_torque_curve_at_knee);
  RUN_TEST(test_torque_curve_midpoint);
  RUN_TEST(test_torque_curve_at_max);
  RUN_TEST(test_torque_curve_above_max);
  RUN_TEST(test_torque_curve_degenerate_vmax_less_than_knee);
  RUN_TEST(test_torque_curve_null_config);

  /* Net Torque Margin Tests */
  RUN_TEST(test_net_torque_no_load);
  RUN_TEST(test_net_torque_aiding_load);
  RUN_TEST(test_net_torque_opposing_sufficient);
  RUN_TEST(test_net_torque_deficit);

  /* Freewheeling & Dynamic Slip Velocity Tests */
  RUN_TEST(test_freewheel_velocity_zero_tension);
  RUN_TEST(test_freewheel_velocity_proportional);
  RUN_TEST(test_freewheel_velocity_clamping);
  RUN_TEST(test_slip_velocity_holding_torque);
  RUN_TEST(test_slip_velocity_exceeding_holding_torque);
  RUN_TEST(test_slip_velocity_at_dynamic_torque);

  /* Motion Planning & Step Emission Tests */
  RUN_TEST(test_planner_nominal_tracking_forward);
  RUN_TEST(test_planner_nominal_tracking_small_error);
  RUN_TEST(test_planner_nominal_tracking_reverse);
  RUN_TEST(test_planner_feedforward_rate);
  RUN_TEST(test_planner_torque_deficit_stall_and_slip);
  RUN_TEST(test_planner_stall_trip_trigger_event);
  RUN_TEST(test_planner_overrunning_aiding_tension_slip_and_stall);
  RUN_TEST(test_planner_freewheeling_under_tension);
  RUN_TEST(test_planner_freewheeling_zero_tension_stops);
  RUN_TEST(test_planner_at_target_zero_error_stable);
  RUN_TEST(test_planner_at_target_zero_error_tension_exceeds_holding_torque);
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
  RUN_TEST(test_motion_calc_step_timeout_ms);
  RUN_TEST(test_motion_should_start);
  RUN_TEST(test_motion_should_stop);
  RUN_TEST(test_motion_plan_and_emit_chunk);
  RUN_TEST(test_motion_prepare_start);

  return UNITY_END();
}

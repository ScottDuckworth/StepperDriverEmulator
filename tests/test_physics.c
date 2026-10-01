#include "unity.h"
#include "motion_math.h"
#include "emulator_config.h"

static EmulatorConfig_t test_config;

void setUp(void) {
  test_config = (EmulatorConfig_t) DEFAULT_EMULATOR_CONFIG;
  // Default values:
  // torque_t0 = 1000
  // torque_v_knee = 1000
  // torque_v_max = 8000
  // torque_t_min = 200
  // kfree = 0.005f
}

void tearDown(void) {}

void test_torque_curve_standstill(void) {
  TEST_ASSERT_EQUAL_INT32(1000, CalcMotorTorqueConfig(&test_config, 0.0f));
}

void test_torque_curve_below_knee(void) {
  TEST_ASSERT_EQUAL_INT32(1000, CalcMotorTorqueConfig(&test_config, 500.0f));
}

void test_torque_curve_at_knee(void) {
  TEST_ASSERT_EQUAL_INT32(1000, CalcMotorTorqueConfig(&test_config, 1000.0f));
}

void test_torque_curve_midpoint(void) {
  // Midpoint between v_knee (1000) and v_max (8000) is 4500
  // Torque should be midpoint between t0 (1000) and t_min (200), which is 600
  TEST_ASSERT_EQUAL_INT32(600, CalcMotorTorqueConfig(&test_config, 4500.0f));
}

void test_torque_curve_at_max(void) {
  TEST_ASSERT_EQUAL_INT32(200, CalcMotorTorqueConfig(&test_config, 8000.0f));
}

void test_torque_curve_above_max(void) {
  TEST_ASSERT_EQUAL_INT32(200, CalcMotorTorqueConfig(&test_config, 12000.0f));
  TEST_ASSERT_EQUAL_INT32(200, CalcMotorTorqueConfig(&test_config, 50000.0f));
}

void test_torque_curve_degenerate_vmax_less_than_knee(void) {
  test_config.torque_v_knee = 3000;
  test_config.torque_v_max = 2000; // Inverted / degenerate
  // Below knee: should still return t0
  TEST_ASSERT_EQUAL_INT32(1000, CalcMotorTorqueConfig(&test_config, 1000.0f));
  // At or above knee: should return t_min without division by zero
  TEST_ASSERT_EQUAL_INT32(200, CalcMotorTorqueConfig(&test_config, 3500.0f));
}

void test_torque_curve_null_config(void) {
  TEST_ASSERT_EQUAL_INT32(0, CalcMotorTorqueConfig(NULL, 1000.0f));
}

void test_net_torque_no_load(void) {
  int32_t t_motor = 1000;
  // Forward motion under zero tension
  TEST_ASSERT_EQUAL_INT32(1000, CalcNetTorque(t_motor, 1, 0));
  // Reverse motion under zero tension
  TEST_ASSERT_EQUAL_INT32(1000, CalcNetTorque(t_motor, -1, 0));
}

void test_net_torque_aiding_load(void) {
  int32_t t_motor = 1000;
  // Forward motion assisted by positive tension
  TEST_ASSERT_EQUAL_INT32(1500, CalcNetTorque(t_motor, 1, 500));
  // Reverse motion assisted by negative tension
  TEST_ASSERT_EQUAL_INT32(1500, CalcNetTorque(t_motor, -1, -500));
}

void test_net_torque_opposing_sufficient(void) {
  int32_t t_motor = 1000;
  // Forward motion opposed by 600 tension -> net torque is positive (+400)
  TEST_ASSERT_EQUAL_INT32(400, CalcNetTorque(t_motor, 1, -600));
  // Reverse motion opposed by 600 tension -> net torque is positive (+400)
  TEST_ASSERT_EQUAL_INT32(400, CalcNetTorque(t_motor, -1, 600));
}

void test_net_torque_deficit(void) {
  int32_t t_motor = 1000;
  // Forward motion opposed by 1500 tension -> net torque is negative (-500)
  TEST_ASSERT_EQUAL_INT32(-500, CalcNetTorque(t_motor, 1, -1500));
  // Reverse motion opposed by 1500 tension -> net torque is negative (-500)
  TEST_ASSERT_EQUAL_INT32(-500, CalcNetTorque(t_motor, -1, 1500));
}

void test_freewheel_velocity_zero_tension(void) {
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, CalcFreewheelVelocity(&test_config, 0));
}

void test_freewheel_velocity_proportional(void) {
  test_config.kfree = 0.005f;
  // 1000 * 0.005 = 5.0 counts/sec
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 5.0f, CalcFreewheelVelocity(&test_config, 1000));
  // -1000 * 0.005 = -5.0 counts/sec
  TEST_ASSERT_FLOAT_WITHIN(0.001f, -5.0f, CalcFreewheelVelocity(&test_config, -1000));
}

void test_freewheel_velocity_clamping(void) {
  test_config.torque_v_max = 8000;
  test_config.kfree = 0.005f;
  // Large tension would produce 50,000 counts/sec -> clamped to +8000
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 8000.0f, CalcFreewheelVelocity(&test_config, 10000000));
  // Negative large tension -> clamped to -8000
  TEST_ASSERT_FLOAT_WITHIN(0.001f, -8000.0f, CalcFreewheelVelocity(&test_config, -10000000));
}

void test_slip_velocity_holding_torque(void) {
  test_config.torque_t0 = 1000;
  test_config.kfree = 0.005f;

  // Below holding torque shelf -> rotor does not slip (returns 0.0f)
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, CalcSlipVelocity(&test_config, 500));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, CalcSlipVelocity(&test_config, -500));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, CalcSlipVelocity(&test_config, 1000));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, CalcSlipVelocity(&test_config, -1000));
}

void test_slip_velocity_exceeding_holding_torque(void) {
  test_config.torque_t0 = 1000;
  test_config.torque_v_max = 8000;
  test_config.kfree = 0.005f;

  // Tension 3000 exceeds t0 (1000) by 2000 -> slip = 2000 * 0.005 = 10.0
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 10.0f, CalcSlipVelocity(&test_config, 3000));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 10.0f, CalcSlipVelocity(&test_config, -3000));

  // Huge tension -> clamped to v_max (8000)
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 8000.0f, CalcSlipVelocity(&test_config, 5000000));
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_torque_curve_standstill);
  RUN_TEST(test_torque_curve_below_knee);
  RUN_TEST(test_torque_curve_at_knee);
  RUN_TEST(test_torque_curve_midpoint);
  RUN_TEST(test_torque_curve_at_max);
  RUN_TEST(test_torque_curve_above_max);
  RUN_TEST(test_torque_curve_degenerate_vmax_less_than_knee);
  RUN_TEST(test_torque_curve_null_config);
  RUN_TEST(test_net_torque_no_load);
  RUN_TEST(test_net_torque_aiding_load);
  RUN_TEST(test_net_torque_opposing_sufficient);
  RUN_TEST(test_net_torque_deficit);
  RUN_TEST(test_freewheel_velocity_zero_tension);
  RUN_TEST(test_freewheel_velocity_proportional);
  RUN_TEST(test_freewheel_velocity_clamping);
  RUN_TEST(test_slip_velocity_holding_torque);
  RUN_TEST(test_slip_velocity_exceeding_holding_torque);
  return UNITY_END();
}

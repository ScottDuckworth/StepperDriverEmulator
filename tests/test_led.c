#include "unity.h"
#include "led_control.h"

void setUp(void) {}
void tearDown(void) {}

void test_led_priority1_blink_mode(void) {
  uint8_t r = 0, g = 0;

  // Phase 0 (0 to 124 ms): Off
  EvalLEDState(0, true, true, false, 0, &r, &g);
  TEST_ASSERT_EQUAL_UINT8(0, r);
  TEST_ASSERT_EQUAL_UINT8(0, g);

  EvalLEDState(124, true, false, false, 0, &r, &g);
  TEST_ASSERT_EQUAL_UINT8(0, r);
  TEST_ASSERT_EQUAL_UINT8(0, g);

  // Phase 1 (125 to 249 ms): Yellow (R=255, G=255)
  EvalLEDState(125, true, true, true, 0, &r, &g);
  TEST_ASSERT_EQUAL_UINT8(255, r);
  TEST_ASSERT_EQUAL_UINT8(255, g);

  EvalLEDState(249, true, true, true, 0, &r, &g);
  TEST_ASSERT_EQUAL_UINT8(255, r);
  TEST_ASSERT_EQUAL_UINT8(255, g);

  // Phase 2 (250 to 374 ms): Off
  EvalLEDState(250, true, false, true, 0, &r, &g);
  TEST_ASSERT_EQUAL_UINT8(0, r);
  TEST_ASSERT_EQUAL_UINT8(0, g);

  // Phase 3 (375 to 499 ms): Yellow
  EvalLEDState(375, true, false, true, 0, &r, &g);
  TEST_ASSERT_EQUAL_UINT8(255, r);
  TEST_ASSERT_EQUAL_UINT8(255, g);
}

void test_led_priority2_stall_tripped(void) {
  uint8_t r = 0, g = 0;

  // Phase 1: Red only (R=255, G=0)
  EvalLEDState(125, false, true, false, 0, &r, &g);
  TEST_ASSERT_EQUAL_UINT8(255, r);
  TEST_ASSERT_EQUAL_UINT8(0, g);

  // Phase 0: Off
  EvalLEDState(0, false, true, true, 0, &r, &g);
  TEST_ASSERT_EQUAL_UINT8(0, r);
  TEST_ASSERT_EQUAL_UINT8(0, g);

  // Even if step_enabled is false or stepping active, stall_tripped overrides
  EvalLEDState(125, false, true, false, 120, &r, &g);
  TEST_ASSERT_EQUAL_UINT8(255, r);
  TEST_ASSERT_EQUAL_UINT8(0, g);
}

void test_led_priority3_step_disabled(void) {
  uint8_t r = 0, g = 0;

  // Step disabled and no faults: LED must be completely off
  EvalLEDState(0, false, false, false, 0, &r, &g);
  TEST_ASSERT_EQUAL_UINT8(0, r);
  TEST_ASSERT_EQUAL_UINT8(0, g);

  EvalLEDState(125, false, false, false, 0, &r, &g);
  TEST_ASSERT_EQUAL_UINT8(0, r);
  TEST_ASSERT_EQUAL_UINT8(0, g);

  EvalLEDState(250, false, false, false, 240, &r, &g);
  TEST_ASSERT_EQUAL_UINT8(0, r);
  TEST_ASSERT_EQUAL_UINT8(0, g);
}

void test_led_priority4_stepping_active(void) {
  uint8_t r = 0, g = 0;

  // Stepping active when (now - last_step_time) < 200 ms
  // Phase 1 (125 ms): Green blinking on (R=0, G=255)
  EvalLEDState(125, false, false, true, 100, &r, &g); // diff = 25 ms < 200
  TEST_ASSERT_EQUAL_UINT8(0, r);
  TEST_ASSERT_EQUAL_UINT8(255, g);

  // Phase 0 (0 ms or 250 ms): Green blinking off (R=0, G=0)
  EvalLEDState(250, false, false, true, 100, &r, &g); // diff = 150 ms < 200
  TEST_ASSERT_EQUAL_UINT8(0, r);
  TEST_ASSERT_EQUAL_UINT8(0, g);

  // Near boundary: diff = 199 ms -> still active
  EvalLEDState(375, false, false, true, 176, &r, &g); // 375 - 176 = 199 < 200
  TEST_ASSERT_EQUAL_UINT8(0, r);
  TEST_ASSERT_EQUAL_UINT8(255, g);
}

void test_led_priority5_stepping_idle(void) {
  uint8_t r = 0, g = 0;

  // Stepping idle when (now - last_step_time) >= 200 ms: Solid Green in all phases
  // Exact boundary: diff = 200 ms
  EvalLEDState(200, false, false, true, 0, &r, &g);
  TEST_ASSERT_EQUAL_UINT8(0, r);
  TEST_ASSERT_EQUAL_UINT8(255, g);

  // Phase 0: Solid Green
  EvalLEDState(250, false, false, true, 0, &r, &g);
  TEST_ASSERT_EQUAL_UINT8(0, r);
  TEST_ASSERT_EQUAL_UINT8(255, g);

  // Phase 1: Solid Green
  EvalLEDState(375, false, false, true, 0, &r, &g);
  TEST_ASSERT_EQUAL_UINT8(0, r);
  TEST_ASSERT_EQUAL_UINT8(255, g);
}

void test_led_null_pointer_safety(void) {
  uint8_t r = 99;
  uint8_t g = 99;

  // Passing NULL outputs must not crash
  EvalLEDState(125, true, false, true, 0, NULL, NULL);

  EvalLEDState(125, true, false, true, 0, &r, NULL);
  TEST_ASSERT_EQUAL_UINT8(255, r);

  EvalLEDState(125, true, false, true, 0, NULL, &g);
  TEST_ASSERT_EQUAL_UINT8(255, g);
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_led_priority1_blink_mode);
  RUN_TEST(test_led_priority2_stall_tripped);
  RUN_TEST(test_led_priority3_step_disabled);
  RUN_TEST(test_led_priority4_stepping_active);
  RUN_TEST(test_led_priority5_stepping_idle);
  RUN_TEST(test_led_null_pointer_safety);
  return UNITY_END();
}

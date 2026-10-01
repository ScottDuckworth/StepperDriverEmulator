#include "unity.h"
#include "quadrature.h"
#include <stdint.h>
#include <stdbool.h>

void setUp(void) {}
void tearDown(void) {}

void test_next_quad_state_forward(void) {
  // Forward progression: 0 -> 1 -> 2 -> 3 -> 0
  uint8_t state = 0;
  state = NextQuadState(state, 1);
  TEST_ASSERT_EQUAL_UINT8(1, state);
  state = NextQuadState(state, 1);
  TEST_ASSERT_EQUAL_UINT8(2, state);
  state = NextQuadState(state, 1);
  TEST_ASSERT_EQUAL_UINT8(3, state);
  state = NextQuadState(state, 1);
  TEST_ASSERT_EQUAL_UINT8(0, state);

  // Any positive direction value acts as forward
  TEST_ASSERT_EQUAL_UINT8(1, NextQuadState(0, 5));
  TEST_ASSERT_EQUAL_UINT8(2, NextQuadState(1, 100));
}

void test_next_quad_state_reverse(void) {
  // Reverse progression: 0 -> 3 -> 2 -> 1 -> 0
  uint8_t state = 0;
  state = NextQuadState(state, -1);
  TEST_ASSERT_EQUAL_UINT8(3, state);
  state = NextQuadState(state, -1);
  TEST_ASSERT_EQUAL_UINT8(2, state);
  state = NextQuadState(state, -1);
  TEST_ASSERT_EQUAL_UINT8(1, state);
  state = NextQuadState(state, -1);
  TEST_ASSERT_EQUAL_UINT8(0, state);

  // Any negative direction value acts as reverse
  TEST_ASSERT_EQUAL_UINT8(3, NextQuadState(0, -5));
  TEST_ASSERT_EQUAL_UINT8(0, NextQuadState(1, -100));
}

void test_next_quad_state_zero_dir(void) {
  // Direction 0 holds current state
  TEST_ASSERT_EQUAL_UINT8(0, NextQuadState(0, 0));
  TEST_ASSERT_EQUAL_UINT8(1, NextQuadState(1, 0));
  TEST_ASSERT_EQUAL_UINT8(2, NextQuadState(2, 0));
  TEST_ASSERT_EQUAL_UINT8(3, NextQuadState(3, 0));
}

void test_next_quad_state_wrapping_and_masking(void) {
  // Input states >= 4 are masked with & 3
  TEST_ASSERT_EQUAL_UINT8(0, NextQuadState(4, 0));
  TEST_ASSERT_EQUAL_UINT8(1, NextQuadState(4, 1));
  TEST_ASSERT_EQUAL_UINT8(3, NextQuadState(4, -1));

  TEST_ASSERT_EQUAL_UINT8(2, NextQuadState(5, 1)); // (5+1)&3 = 2
  TEST_ASSERT_EQUAL_UINT8(0, NextQuadState(5, -1)); // (5-1)&3 = 0
}

void test_quad_bsrr_values(void) {
  // State 0: EA=0, EB=0 -> BR_4 | BR_5
  uint32_t s0 = GetQuadBsrrValue(0);
  TEST_ASSERT_EQUAL_HEX32(GPIO_BSRR_BR_4 | GPIO_BSRR_BR_5, s0);
  TEST_ASSERT_EQUAL_UINT32(0, s0 & (GPIO_BSRR_BS_4 | GPIO_BSRR_BS_5));

  // State 1: EA=1, EB=0 -> BS_4 | BR_5
  uint32_t s1 = GetQuadBsrrValue(1);
  TEST_ASSERT_EQUAL_HEX32(GPIO_BSRR_BS_4 | GPIO_BSRR_BR_5, s1);
  TEST_ASSERT_TRUE((s1 & GPIO_BSRR_BS_4) != 0);
  TEST_ASSERT_TRUE((s1 & GPIO_BSRR_BR_5) != 0);
  TEST_ASSERT_EQUAL_UINT32(0, s1 & (GPIO_BSRR_BR_4 | GPIO_BSRR_BS_5));

  // State 2: EA=1, EB=1 -> BS_4 | BS_5
  uint32_t s2 = GetQuadBsrrValue(2);
  TEST_ASSERT_EQUAL_HEX32(GPIO_BSRR_BS_4 | GPIO_BSRR_BS_5, s2);
  TEST_ASSERT_TRUE((s2 & GPIO_BSRR_BS_4) != 0);
  TEST_ASSERT_TRUE((s2 & GPIO_BSRR_BS_5) != 0);
  TEST_ASSERT_EQUAL_UINT32(0, s2 & (GPIO_BSRR_BR_4 | GPIO_BSRR_BR_5));

  // State 3: EA=0, EB=1 -> BR_4 | BS_5
  uint32_t s3 = GetQuadBsrrValue(3);
  TEST_ASSERT_EQUAL_HEX32(GPIO_BSRR_BR_4 | GPIO_BSRR_BS_5, s3);
  TEST_ASSERT_TRUE((s3 & GPIO_BSRR_BR_4) != 0);
  TEST_ASSERT_TRUE((s3 & GPIO_BSRR_BS_5) != 0);
  TEST_ASSERT_EQUAL_UINT32(0, s3 & (GPIO_BSRR_BS_4 | GPIO_BSRR_BR_5));

  // Masking for values >= 4
  TEST_ASSERT_EQUAL_HEX32(s0, GetQuadBsrrValue(4));
  TEST_ASSERT_EQUAL_HEX32(s1, GetQuadBsrrValue(5));
}

void test_gray_code_hamming_distance(void) {
  // Test 1-bit change per transition across forward cycles
  uint8_t state = 0;
  for (int step = 0; step < 16; step++) {
    uint8_t next_state = NextQuadState(state, 1);
    uint32_t curr_bsrr = GetQuadBsrrValue(state);
    uint32_t next_bsrr = GetQuadBsrrValue(next_state);

    // Extract pin levels (0 or 1)
    int curr_ea = (curr_bsrr & GPIO_BSRR_BS_4) ? 1 : 0;
    int curr_eb = (curr_bsrr & GPIO_BSRR_BS_5) ? 1 : 0;
    int next_ea = (next_bsrr & GPIO_BSRR_BS_4) ? 1 : 0;
    int next_eb = (next_bsrr & GPIO_BSRR_BS_5) ? 1 : 0;

    int hamming_dist = (curr_ea != next_ea) + (curr_eb != next_eb);
    TEST_ASSERT_EQUAL_INT(1, hamming_dist);

    state = next_state;
  }

  // Test 1-bit change per transition across reverse cycles
  for (int step = 0; step < 16; step++) {
    uint8_t next_state = NextQuadState(state, -1);
    uint32_t curr_bsrr = GetQuadBsrrValue(state);
    uint32_t next_bsrr = GetQuadBsrrValue(next_state);

    int curr_ea = (curr_bsrr & GPIO_BSRR_BS_4) ? 1 : 0;
    int curr_eb = (curr_bsrr & GPIO_BSRR_BS_5) ? 1 : 0;
    int next_ea = (next_bsrr & GPIO_BSRR_BS_4) ? 1 : 0;
    int next_eb = (next_bsrr & GPIO_BSRR_BS_5) ? 1 : 0;

    int hamming_dist = (curr_ea != next_ea) + (curr_eb != next_eb);
    TEST_ASSERT_EQUAL_INT(1, hamming_dist);

    state = next_state;
  }
}

void test_generate_quad_chunk_full_forward(void) {
  uint32_t chunk[8] = {0};
  uint8_t state = 0;

  int8_t delta = GenerateQuadChunk(chunk, 8, &state, 1, 8);
  TEST_ASSERT_EQUAL_INT8(8, delta);
  TEST_ASSERT_EQUAL_UINT8(0, state); // 8 steps from 0 returns to 0

  // Expected sequence: 1, 2, 3, 0, 1, 2, 3, 0
  uint8_t expected_states[8] = {1, 2, 3, 0, 1, 2, 3, 0};
  for (int i = 0; i < 8; i++) {
    TEST_ASSERT_EQUAL_HEX32(GetQuadBsrrValue(expected_states[i]), chunk[i]);
  }
}

void test_generate_quad_chunk_full_reverse(void) {
  uint32_t chunk[8] = {0};
  uint8_t state = 0;

  int8_t delta = GenerateQuadChunk(chunk, 8, &state, -1, 8);
  TEST_ASSERT_EQUAL_INT8(-8, delta);
  TEST_ASSERT_EQUAL_UINT8(0, state);

  // Expected reverse sequence: 3, 2, 1, 0, 3, 2, 1, 0
  uint8_t expected_states[8] = {3, 2, 1, 0, 3, 2, 1, 0};
  for (int i = 0; i < 8; i++) {
    TEST_ASSERT_EQUAL_HEX32(GetQuadBsrrValue(expected_states[i]), chunk[i]);
  }
}

void test_generate_quad_chunk_partial_with_tail_padding(void) {
  uint32_t chunk[8] = {0};
  uint8_t state = 0;

  // Emit 3 forward steps into an 8-slot chunk
  int8_t delta = GenerateQuadChunk(chunk, 8, &state, 1, 3);
  TEST_ASSERT_EQUAL_INT8(3, delta);
  TEST_ASSERT_EQUAL_UINT8(3, state);

  // Active steps: 1, 2, 3
  TEST_ASSERT_EQUAL_HEX32(GetQuadBsrrValue(1), chunk[0]);
  TEST_ASSERT_EQUAL_HEX32(GetQuadBsrrValue(2), chunk[1]);
  TEST_ASSERT_EQUAL_HEX32(GetQuadBsrrValue(3), chunk[2]);

  // Tail slots (3..7) must hold the last emitted state (state 3)
  for (int i = 3; i < 8; i++) {
    TEST_ASSERT_EQUAL_HEX32(GetQuadBsrrValue(3), chunk[i]);
  }
}

void test_generate_quad_chunk_partial_reverse(void) {
  uint32_t chunk[8] = {0};
  uint8_t state = 2;

  // Emit 2 reverse steps into an 8-slot chunk starting at state 2
  int8_t delta = GenerateQuadChunk(chunk, 8, &state, -1, 2);
  TEST_ASSERT_EQUAL_INT8(-2, delta);
  TEST_ASSERT_EQUAL_UINT8(0, state);

  // Active steps: 1, 0
  TEST_ASSERT_EQUAL_HEX32(GetQuadBsrrValue(1), chunk[0]);
  TEST_ASSERT_EQUAL_HEX32(GetQuadBsrrValue(0), chunk[1]);

  // Tail slots (2..7) must hold state 0
  for (int i = 2; i < 8; i++) {
    TEST_ASSERT_EQUAL_HEX32(GetQuadBsrrValue(0), chunk[i]);
  }
}

void test_generate_quad_chunk_zero_emit_or_dir(void) {
  uint32_t chunk[4] = {0};
  uint8_t state = 2;

  // Zero count_to_emit: all slots padded with current state (2), state unchanged
  int8_t delta = GenerateQuadChunk(chunk, 4, &state, 1, 0);
  TEST_ASSERT_EQUAL_INT8(0, delta);
  TEST_ASSERT_EQUAL_UINT8(2, state);
  for (int i = 0; i < 4; i++) {
    TEST_ASSERT_EQUAL_HEX32(GetQuadBsrrValue(2), chunk[i]);
  }

  // Dir == 0: delta = 0, state unchanged
  delta = GenerateQuadChunk(chunk, 4, &state, 0, 3);
  TEST_ASSERT_EQUAL_INT8(0, delta);
  TEST_ASSERT_EQUAL_UINT8(2, state);
}

void test_generate_quad_chunk_clamping_and_null_protection(void) {
  uint32_t chunk[4];
  uint8_t state = 0;

  // NULL buffer safety
  TEST_ASSERT_EQUAL_INT8(0, GenerateQuadChunk(NULL, 4, &state, 1, 2));

  // NULL state pointer safety
  TEST_ASSERT_EQUAL_INT8(0, GenerateQuadChunk(chunk, 4, NULL, 1, 2));

  // Zero chunk size
  TEST_ASSERT_EQUAL_INT8(0, GenerateQuadChunk(chunk, 0, &state, 1, 2));

  // count_to_emit > chunk_size clamped to chunk_size
  state = 0;
  int8_t delta = GenerateQuadChunk(chunk, 4, &state, 1, 20);
  TEST_ASSERT_EQUAL_INT8(4, delta);
  TEST_ASSERT_EQUAL_UINT8(0, state); // 4 steps from 0 wraps back to 0
}

void test_calc_timer_pacing_nominal_rates(void) {
  uint16_t psc = 999;
  uint16_t arr = 999;

  // 100 kHz (100.0 counts/ms) -> 48000 / 100 = 480 ticks
  CalcTimerPacing(100.0f, &psc, &arr);
  TEST_ASSERT_EQUAL_UINT16(0, psc);
  TEST_ASSERT_EQUAL_UINT16(479, arr);

  // 50 kHz (50.0 counts/ms) -> 48000 / 50 = 960 ticks
  CalcTimerPacing(50.0f, &psc, &arr);
  TEST_ASSERT_EQUAL_UINT16(0, psc);
  TEST_ASSERT_EQUAL_UINT16(959, arr);

  // 10 kHz (10.0 counts/ms) -> 48000 / 10 = 4800 ticks
  CalcTimerPacing(10.0f, &psc, &arr);
  TEST_ASSERT_EQUAL_UINT16(0, psc);
  TEST_ASSERT_EQUAL_UINT16(4799, arr);

  // 1 kHz (1.0 counts/ms) -> 48000 / 1 = 48000 ticks
  CalcTimerPacing(1.0f, &psc, &arr);
  TEST_ASSERT_EQUAL_UINT16(0, psc);
  TEST_ASSERT_EQUAL_UINT16(47999, arr);
}

void test_calc_timer_pacing_ceiling_and_floor(void) {
  uint16_t psc = 999;
  uint16_t arr = 999;

  // Ceiling: > 100 kHz clamped to 100 kHz (480 ticks, ARR=479)
  CalcTimerPacing(150.0f, &psc, &arr);
  TEST_ASSERT_EQUAL_UINT16(0, psc);
  TEST_ASSERT_EQUAL_UINT16(479, arr);

  // Floor: 0.1 counts/ms (100 Hz)
  // 48000 / 0.1 = 480000 ticks.
  // psc = 480000 >> 16 = 7.
  // ticks = 480000 / (7 + 1) = 60000.
  // arr = 59999.
  CalcTimerPacing(0.1f, &psc, &arr);
  TEST_ASSERT_EQUAL_UINT16(7, psc);
  TEST_ASSERT_EQUAL_UINT16(59999, arr);

  // Below floor (0.01 counts/ms or 0.0) is clamped to 0.1 floor
  CalcTimerPacing(0.01f, &psc, &arr);
  TEST_ASSERT_EQUAL_UINT16(7, psc);
  TEST_ASSERT_EQUAL_UINT16(59999, arr);

  CalcTimerPacing(0.0f, &psc, &arr);
  TEST_ASSERT_EQUAL_UINT16(7, psc);
  TEST_ASSERT_EQUAL_UINT16(59999, arr);
}

void test_calc_timer_pacing_negative_velocity(void) {
  uint16_t psc = 999;
  uint16_t arr = 999;

  // Negative velocities take absolute value
  CalcTimerPacing(-10.0f, &psc, &arr);
  TEST_ASSERT_EQUAL_UINT16(0, psc);
  TEST_ASSERT_EQUAL_UINT16(4799, arr);

  CalcTimerPacing(-0.05f, &psc, &arr);
  TEST_ASSERT_EQUAL_UINT16(7, psc);
  TEST_ASSERT_EQUAL_UINT16(59999, arr);
}

void test_calc_timer_pacing_null_pointers(void) {
  uint16_t psc = 999;
  uint16_t arr = 999;

  // Should safely execute without null pointer dereferences
  CalcTimerPacing(10.0f, NULL, NULL);

  CalcTimerPacing(10.0f, &psc, NULL);
  TEST_ASSERT_EQUAL_UINT16(0, psc);

  CalcTimerPacing(10.0f, NULL, &arr);
  TEST_ASSERT_EQUAL_UINT16(4799, arr);
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_next_quad_state_forward);
  RUN_TEST(test_next_quad_state_reverse);
  RUN_TEST(test_next_quad_state_zero_dir);
  RUN_TEST(test_next_quad_state_wrapping_and_masking);
  RUN_TEST(test_quad_bsrr_values);
  RUN_TEST(test_gray_code_hamming_distance);
  RUN_TEST(test_generate_quad_chunk_full_forward);
  RUN_TEST(test_generate_quad_chunk_full_reverse);
  RUN_TEST(test_generate_quad_chunk_partial_with_tail_padding);
  RUN_TEST(test_generate_quad_chunk_partial_reverse);
  RUN_TEST(test_generate_quad_chunk_zero_emit_or_dir);
  RUN_TEST(test_generate_quad_chunk_clamping_and_null_protection);
  RUN_TEST(test_calc_timer_pacing_nominal_rates);
  RUN_TEST(test_calc_timer_pacing_ceiling_and_floor);
  RUN_TEST(test_calc_timer_pacing_negative_velocity);
  RUN_TEST(test_calc_timer_pacing_null_pointers);
  return UNITY_END();
}

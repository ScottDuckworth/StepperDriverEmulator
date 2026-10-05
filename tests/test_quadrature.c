#include "unity.h"
#include "quadrature.h"
#include <stdint.h>
#include <stdbool.h>

void setUp(void) {}
void tearDown(void) {}

void test_next_quad_state_forward(void) {
  // Forward progression: 0 -> 1 -> 2 -> 3 -> 0
  uint8_t state = 0;
  state = Quadrature_NextState(state, 1);
  TEST_ASSERT_EQUAL_UINT8(1, state);
  state = Quadrature_NextState(state, 1);
  TEST_ASSERT_EQUAL_UINT8(2, state);
  state = Quadrature_NextState(state, 1);
  TEST_ASSERT_EQUAL_UINT8(3, state);
  state = Quadrature_NextState(state, 1);
  TEST_ASSERT_EQUAL_UINT8(0, state);

  // Any positive direction value acts as forward
  TEST_ASSERT_EQUAL_UINT8(1, Quadrature_NextState(0, 5));
  TEST_ASSERT_EQUAL_UINT8(2, Quadrature_NextState(1, 100));
}

void test_next_quad_state_reverse(void) {
  // Reverse progression: 0 -> 3 -> 2 -> 1 -> 0
  uint8_t state = 0;
  state = Quadrature_NextState(state, -1);
  TEST_ASSERT_EQUAL_UINT8(3, state);
  state = Quadrature_NextState(state, -1);
  TEST_ASSERT_EQUAL_UINT8(2, state);
  state = Quadrature_NextState(state, -1);
  TEST_ASSERT_EQUAL_UINT8(1, state);
  state = Quadrature_NextState(state, -1);
  TEST_ASSERT_EQUAL_UINT8(0, state);

  // Any negative direction value acts as reverse
  TEST_ASSERT_EQUAL_UINT8(3, Quadrature_NextState(0, -5));
  TEST_ASSERT_EQUAL_UINT8(0, Quadrature_NextState(1, -100));
}

void test_next_quad_state_zero_dir(void) {
  // Direction 0 holds current state
  TEST_ASSERT_EQUAL_UINT8(0, Quadrature_NextState(0, 0));
  TEST_ASSERT_EQUAL_UINT8(1, Quadrature_NextState(1, 0));
  TEST_ASSERT_EQUAL_UINT8(2, Quadrature_NextState(2, 0));
  TEST_ASSERT_EQUAL_UINT8(3, Quadrature_NextState(3, 0));
}

void test_next_quad_state_wrapping_and_masking(void) {
  // Input states >= 4 are masked with & 3
  TEST_ASSERT_EQUAL_UINT8(0, Quadrature_NextState(4, 0));
  TEST_ASSERT_EQUAL_UINT8(1, Quadrature_NextState(4, 1));
  TEST_ASSERT_EQUAL_UINT8(3, Quadrature_NextState(4, -1));

  TEST_ASSERT_EQUAL_UINT8(2, Quadrature_NextState(5, 1)); // (5+1)&3 = 2
  TEST_ASSERT_EQUAL_UINT8(0, Quadrature_NextState(5, -1)); // (5-1)&3 = 0
}

void test_quad_bsrr_values(void) {
  // State 0: EA=0, EB=0 -> BR_4 | BR_5
  uint32_t s0 = Quadrature_GetBsrrValue(0);
  TEST_ASSERT_EQUAL_HEX32(GPIO_BSRR_BR_4 | GPIO_BSRR_BR_5, s0);
  TEST_ASSERT_EQUAL_UINT32(0, s0 & (GPIO_BSRR_BS_4 | GPIO_BSRR_BS_5));

  // State 1: EA=1, EB=0 -> BS_4 | BR_5
  uint32_t s1 = Quadrature_GetBsrrValue(1);
  TEST_ASSERT_EQUAL_HEX32(GPIO_BSRR_BS_4 | GPIO_BSRR_BR_5, s1);
  TEST_ASSERT_TRUE((s1 & GPIO_BSRR_BS_4) != 0);
  TEST_ASSERT_TRUE((s1 & GPIO_BSRR_BR_5) != 0);
  TEST_ASSERT_EQUAL_UINT32(0, s1 & (GPIO_BSRR_BR_4 | GPIO_BSRR_BS_5));

  // State 2: EA=1, EB=1 -> BS_4 | BS_5
  uint32_t s2 = Quadrature_GetBsrrValue(2);
  TEST_ASSERT_EQUAL_HEX32(GPIO_BSRR_BS_4 | GPIO_BSRR_BS_5, s2);
  TEST_ASSERT_TRUE((s2 & GPIO_BSRR_BS_4) != 0);
  TEST_ASSERT_TRUE((s2 & GPIO_BSRR_BS_5) != 0);
  TEST_ASSERT_EQUAL_UINT32(0, s2 & (GPIO_BSRR_BR_4 | GPIO_BSRR_BR_5));

  // State 3: EA=0, EB=1 -> BR_4 | BS_5
  uint32_t s3 = Quadrature_GetBsrrValue(3);
  TEST_ASSERT_EQUAL_HEX32(GPIO_BSRR_BR_4 | GPIO_BSRR_BS_5, s3);
  TEST_ASSERT_TRUE((s3 & GPIO_BSRR_BR_4) != 0);
  TEST_ASSERT_TRUE((s3 & GPIO_BSRR_BS_5) != 0);
  TEST_ASSERT_EQUAL_UINT32(0, s3 & (GPIO_BSRR_BS_4 | GPIO_BSRR_BR_5));

  // Masking for values >= 4
  TEST_ASSERT_EQUAL_HEX32(s0, Quadrature_GetBsrrValue(4));
  TEST_ASSERT_EQUAL_HEX32(s1, Quadrature_GetBsrrValue(5));
}

void test_gray_code_hamming_distance(void) {
  // Test 1-bit change per transition across forward cycles
  uint8_t state = 0;
  for (int step = 0; step < 16; step++) {
    uint8_t next_state = Quadrature_NextState(state, 1);
    uint32_t curr_bsrr = Quadrature_GetBsrrValue(state);
    uint32_t next_bsrr = Quadrature_GetBsrrValue(next_state);

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
    uint8_t next_state = Quadrature_NextState(state, -1);
    uint32_t curr_bsrr = Quadrature_GetBsrrValue(state);
    uint32_t next_bsrr = Quadrature_GetBsrrValue(next_state);

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

  int8_t delta = Quadrature_GenerateChunk(chunk, 8, &state, 1, 8);
  TEST_ASSERT_EQUAL_INT8(8, delta);
  TEST_ASSERT_EQUAL_UINT8(0, state); // 8 steps from 0 returns to 0

  // Expected sequence: 1, 2, 3, 0, 1, 2, 3, 0
  uint8_t expected_states[8] = {1, 2, 3, 0, 1, 2, 3, 0};
  for (int i = 0; i < 8; i++) {
    TEST_ASSERT_EQUAL_HEX32(Quadrature_GetBsrrValue(expected_states[i]), chunk[i]);
  }
}

void test_generate_quad_chunk_full_reverse(void) {
  uint32_t chunk[8] = {0};
  uint8_t state = 0;

  int8_t delta = Quadrature_GenerateChunk(chunk, 8, &state, -1, 8);
  TEST_ASSERT_EQUAL_INT8(-8, delta);
  TEST_ASSERT_EQUAL_UINT8(0, state);

  // Expected reverse sequence: 3, 2, 1, 0, 3, 2, 1, 0
  uint8_t expected_states[8] = {3, 2, 1, 0, 3, 2, 1, 0};
  for (int i = 0; i < 8; i++) {
    TEST_ASSERT_EQUAL_HEX32(Quadrature_GetBsrrValue(expected_states[i]), chunk[i]);
  }
}

void test_generate_quad_chunk_partial_with_tail_padding(void) {
  uint32_t chunk[8] = {0};
  uint8_t state = 0;

  // Emit 3 forward steps into an 8-slot chunk
  int8_t delta = Quadrature_GenerateChunk(chunk, 8, &state, 1, 3);
  TEST_ASSERT_EQUAL_INT8(3, delta);
  TEST_ASSERT_EQUAL_UINT8(3, state);

  // Active steps: 1, 2, 3
  TEST_ASSERT_EQUAL_HEX32(Quadrature_GetBsrrValue(1), chunk[0]);
  TEST_ASSERT_EQUAL_HEX32(Quadrature_GetBsrrValue(2), chunk[1]);
  TEST_ASSERT_EQUAL_HEX32(Quadrature_GetBsrrValue(3), chunk[2]);

  // Tail slots (3..7) must hold the last emitted state (state 3)
  for (int i = 3; i < 8; i++) {
    TEST_ASSERT_EQUAL_HEX32(Quadrature_GetBsrrValue(3), chunk[i]);
  }
}

void test_generate_quad_chunk_partial_reverse(void) {
  uint32_t chunk[8] = {0};
  uint8_t state = 2;

  // Emit 2 reverse steps into an 8-slot chunk starting at state 2
  int8_t delta = Quadrature_GenerateChunk(chunk, 8, &state, -1, 2);
  TEST_ASSERT_EQUAL_INT8(-2, delta);
  TEST_ASSERT_EQUAL_UINT8(0, state);

  // Active steps: 1, 0
  TEST_ASSERT_EQUAL_HEX32(Quadrature_GetBsrrValue(1), chunk[0]);
  TEST_ASSERT_EQUAL_HEX32(Quadrature_GetBsrrValue(0), chunk[1]);

  // Tail slots (2..7) must hold state 0
  for (int i = 2; i < 8; i++) {
    TEST_ASSERT_EQUAL_HEX32(Quadrature_GetBsrrValue(0), chunk[i]);
  }
}

void test_generate_quad_chunk_zero_emit_or_dir(void) {
  uint32_t chunk[4] = {0};
  uint8_t state = 2;

  // Zero count_to_emit: all slots padded with current state (2), state unchanged
  int8_t delta = Quadrature_GenerateChunk(chunk, 4, &state, 1, 0);
  TEST_ASSERT_EQUAL_INT8(0, delta);
  TEST_ASSERT_EQUAL_UINT8(2, state);
  for (int i = 0; i < 4; i++) {
    TEST_ASSERT_EQUAL_HEX32(Quadrature_GetBsrrValue(2), chunk[i]);
  }

  // Dir == 0: delta = 0, state unchanged
  delta = Quadrature_GenerateChunk(chunk, 4, &state, 0, 3);
  TEST_ASSERT_EQUAL_INT8(0, delta);
  TEST_ASSERT_EQUAL_UINT8(2, state);
}

void test_generate_quad_chunk_clamping_and_null_protection(void) {
  uint32_t chunk[4];
  uint8_t state = 0;

  // NULL buffer safety
  TEST_ASSERT_EQUAL_INT8(0, Quadrature_GenerateChunk(NULL, 4, &state, 1, 2));

  // NULL state pointer safety
  TEST_ASSERT_EQUAL_INT8(0, Quadrature_GenerateChunk(chunk, 4, NULL, 1, 2));

  // Zero chunk size
  TEST_ASSERT_EQUAL_INT8(0, Quadrature_GenerateChunk(chunk, 0, &state, 1, 2));

  // count_to_emit > chunk_size clamped to chunk_size
  state = 0;
  int8_t delta = Quadrature_GenerateChunk(chunk, 4, &state, 1, 20);
  TEST_ASSERT_EQUAL_INT8(4, delta);
  TEST_ASSERT_EQUAL_UINT8(0, state); // 4 steps from 0 wraps back to 0
}

void test_calc_timer_pacing_nominal_rates(void) {
  uint16_t psc = 999;
  uint16_t arr = 999;

  // 100 kHz (100,000 counts/s) -> 48000000 / 100000 = 480 ticks
  Quadrature_CalcTimerPacing(100000, &psc, &arr);
  TEST_ASSERT_EQUAL_UINT16(0, psc);
  TEST_ASSERT_EQUAL_UINT16(479, arr);

  // 50 kHz (50,000 counts/s) -> 48000000 / 50000 = 960 ticks
  Quadrature_CalcTimerPacing(50000, &psc, &arr);
  TEST_ASSERT_EQUAL_UINT16(0, psc);
  TEST_ASSERT_EQUAL_UINT16(959, arr);

  // 10 kHz (10,000 counts/s) -> 48000000 / 10000 = 4800 ticks
  Quadrature_CalcTimerPacing(10000, &psc, &arr);
  TEST_ASSERT_EQUAL_UINT16(0, psc);
  TEST_ASSERT_EQUAL_UINT16(4799, arr);

  // 1 kHz (1,000 counts/s) -> 48000000 / 1000 = 48000 ticks
  Quadrature_CalcTimerPacing(1000, &psc, &arr);
  TEST_ASSERT_EQUAL_UINT16(0, psc);
  TEST_ASSERT_EQUAL_UINT16(47999, arr);
}

void test_calc_timer_pacing_ceiling_and_floor(void) {
  uint16_t psc = 999;
  uint16_t arr = 999;

  // Ceiling: > 300 kHz clamped to 300 kHz (160 ticks, ARR=159)
  Quadrature_CalcTimerPacing(350000, &psc, &arr);
  TEST_ASSERT_EQUAL_UINT16(0, psc);
  TEST_ASSERT_EQUAL_UINT16(159, arr);

  // 200 kHz (200,000 counts/s) -> 48000000 / 200000 = 240 ticks, ARR=239 (3000 RPM @ 4000 CPR)
  Quadrature_CalcTimerPacing(200000, &psc, &arr);
  TEST_ASSERT_EQUAL_UINT16(0, psc);
  TEST_ASSERT_EQUAL_UINT16(239, arr);

  // 150 kHz (150,000 counts/s) -> 48000000 / 150000 = 320 ticks, ARR=319
  Quadrature_CalcTimerPacing(150000, &psc, &arr);
  TEST_ASSERT_EQUAL_UINT16(0, psc);
  TEST_ASSERT_EQUAL_UINT16(319, arr);

  // 80 Hz quad rate: 48 MHz / 600,000 ticks -> 80 counts/s
  // psc = 600000 >> 16 = 9. ticks = 600000 / 10 = 60000. arr = 59999.
  Quadrature_CalcTimerPacing(80, &psc, &arr);
  TEST_ASSERT_EQUAL_UINT16(9, psc);
  TEST_ASSERT_EQUAL_UINT16(59999, arr);

  // Floor: 1 count/s
  // 48000000 / 1 = 48000000 ticks.
  // psc = 48000000 >> 16 = 732.
  // ticks = 48000000 / (732 + 1) = 65484.
  // arr = 65483.
  Quadrature_CalcTimerPacing(1, &psc, &arr);
  TEST_ASSERT_EQUAL_UINT16(732, psc);
  TEST_ASSERT_EQUAL_UINT16(65483, arr);

  // Zero rate clamped to floor (1 count/s)
  Quadrature_CalcTimerPacing(0, &psc, &arr);
  TEST_ASSERT_EQUAL_UINT16(732, psc);
  TEST_ASSERT_EQUAL_UINT16(65483, arr);
}

void test_calc_timer_pacing_null_pointers(void) {
  uint16_t psc = 999;
  uint16_t arr = 999;

  // Should safely execute without null pointer dereferences
  Quadrature_CalcTimerPacing(10000, NULL, NULL);

  Quadrature_CalcTimerPacing(10000, &psc, NULL);
  TEST_ASSERT_EQUAL_UINT16(0, psc);

  Quadrature_CalcTimerPacing(10000, NULL, &arr);
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
  RUN_TEST(test_calc_timer_pacing_null_pointers);
  return UNITY_END();
}

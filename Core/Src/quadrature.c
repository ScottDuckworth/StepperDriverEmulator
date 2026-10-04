#include "quadrature.h"

static const uint32_t QUAD_BSRR_STATES[4] = {
    GPIO_BSRR_BR_4 | GPIO_BSRR_BR_5, // State 0: EA=0, EB=0
    GPIO_BSRR_BS_4 | GPIO_BSRR_BR_5, // State 1: EA=1, EB=0
    GPIO_BSRR_BS_4 | GPIO_BSRR_BS_5, // State 2: EA=1, EB=1
    GPIO_BSRR_BR_4 | GPIO_BSRR_BS_5  // State 3: EA=0, EB=1
};

uint8_t NextQuadState(uint8_t current_state, int dir) {
  if (dir > 0) {
    return (current_state + 1) & 3;
  } else if (dir < 0) {
    return (current_state - 1) & 3;
  }
  return current_state & 3;
}

uint32_t GetQuadBsrrValue(uint8_t quad_state) {
  return QUAD_BSRR_STATES[quad_state & 3];
}

int8_t GenerateQuadChunk(uint32_t* chunk, uint16_t chunk_size, uint8_t* inout_state, int dir, uint16_t count_to_emit) {
  if (!chunk || !inout_state || chunk_size == 0) return 0;
  if (count_to_emit > chunk_size) {
    count_to_emit = chunk_size;
  }

  uint8_t state = *inout_state;
  if (dir > 0) {
    for (uint16_t i = 0; i < count_to_emit; i++) {
      state = (state + 1) & 3;
      chunk[i] = QUAD_BSRR_STATES[state];
    }
  } else if (dir < 0) {
    for (uint16_t i = 0; i < count_to_emit; i++) {
      state = (state - 1) & 3;
      chunk[i] = QUAD_BSRR_STATES[state];
    }
  }

  uint32_t fill_val = QUAD_BSRR_STATES[state & 3];
  for (uint16_t i = count_to_emit; i < chunk_size; i++) {
    chunk[i] = fill_val;
  }

  *inout_state = state;
  return (dir > 0) ? (int8_t) count_to_emit : (dir < 0) ? (int8_t)(-count_to_emit) : 0;
}

void CalcTimerPacing(uint32_t target_velocity_counts_sec, uint16_t* out_psc, uint16_t* out_arr) {
  uint32_t counts_per_sec = target_velocity_counts_sec;
  if (counts_per_sec < 1) {
    counts_per_sec = 1;
  }
  if (counts_per_sec > 300000U) {
    counts_per_sec = 300000U; // max 300 kHz
  }

  uint32_t ticks = 48000000U / counts_per_sec;
  if (ticks < 160) {
    ticks = 160; // max 300 kHz
  }

  uint32_t psc = 0;
  if (ticks > 65536) {
    psc = (ticks >> 16);
    ticks = ticks / (psc + 1);
  }
  if (ticks > 65536) ticks = 65536;

  if (out_psc) *out_psc = (uint16_t) psc;
  if (out_arr) *out_arr = (uint16_t)(ticks - 1);
}


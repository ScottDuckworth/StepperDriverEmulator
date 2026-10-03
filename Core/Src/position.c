#include "position.h"
#include <stdint.h>

int32_t Position_ConvertStepDeltaToCounts(int32_t step_delta, uint16_t ratio_spr, uint16_t ratio_epr, int32_t* remainder) {
  if (ratio_spr == 0) return 0;
  int32_t rem = remainder ? *remainder : 0;
  int32_t accum = rem + step_delta * (int32_t) ratio_epr;
  int32_t counts = accum / (int32_t) ratio_spr;
  rem = accum % (int32_t) ratio_spr;
  if (remainder) {
    *remainder = rem;
  }
  return counts;
}

uint16_t Position_CalcStepDelta(uint16_t current_hw_cnt, uint16_t prev_hw_cnt) {
  return (uint16_t)(current_hw_cnt - prev_hw_cnt);
}

int64_t Position_AccumulateStepPosition(int64_t current_step_pos, uint16_t delta, bool step_reverse) {
  if (step_reverse) {
    return current_step_pos - (int64_t) delta;
  } else {
    return current_step_pos + (int64_t) delta;
  }
}

void Position_RealignCounters(PositionCounters_t* pos, int64_t target_encoder_pos, const EmulatorConfig_t* cfg, uint16_t current_hw_cnt) {
  if (!pos) return;
  (void) cfg;
  pos->encoder_pos = target_encoder_pos;
  pos->commanded_pos = target_encoder_pos;
  pos->step_rem = 0;
  pos->step_cnt_prev = current_hw_cnt;
  pos->step_dcnt = 0;
}

uint16_t Position_FilterStepWithBlanking(uint32_t period, uint32_t min_blanking_ticks, uint32_t* accum_ticks, uint32_t* out_valid_period) {
  if (!accum_ticks) return 0;
  *accum_ticks += period;
  if (*accum_ticks >= min_blanking_ticks) {
    if (out_valid_period) {
      *out_valid_period = *accum_ticks;
    }
    *accum_ticks = 0;
    return 1;
  }
  return 0;
}

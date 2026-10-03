#include "position_tracker.h"
#include "motion_math.h"

uint16_t CalcStepDelta(uint16_t current_hw_cnt, uint16_t prev_hw_cnt) {
  return (uint16_t)(current_hw_cnt - prev_hw_cnt);
}

int64_t AccumulateStepPosition(int64_t current_step_pos, uint16_t delta, bool step_reverse) {
  if (step_reverse) {
    return current_step_pos - (int64_t) delta;
  } else {
    return current_step_pos + (int64_t) delta;
  }
}

void RealignPositionCounters(PositionCounters_t* pos, int64_t target_encoder_pos, const EmulatorConfig_t* cfg, uint16_t current_hw_cnt) {
  if (!pos) return;
  (void) cfg;
  pos->encoder_pos = target_encoder_pos;
  pos->commanded_pos = target_encoder_pos;
  pos->step_rem = 0;
  pos->step_cnt_prev = current_hw_cnt;
  pos->step_dcnt = 0;
}

uint16_t FilterStepWithBlanking(uint32_t period, uint32_t min_blanking_ticks, uint32_t* accum_ticks, uint32_t* out_valid_period) {
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

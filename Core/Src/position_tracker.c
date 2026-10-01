#include "position_tracker.h"
#include "motion_math.h"

uint16_t CalcStepDelta(uint16_t current_hw_cnt, uint16_t prev_hw_cnt) {
  return (uint16_t)(current_hw_cnt - prev_hw_cnt);
}

int32_t AccumulateStepPosition(int32_t current_step_pos, uint16_t delta, bool step_reverse) {
  if (step_reverse) {
    return current_step_pos - (int32_t) delta;
  } else {
    return current_step_pos + (int32_t) delta;
  }
}

void RealignPositionCounters(PositionCounters_t* pos, int32_t target_encoder_pos, const EmulatorConfig_t* cfg, uint16_t current_hw_cnt) {
  if (!pos) return;
  pos->encoder_pos = target_encoder_pos;
  pos->step_pos = EncoderToStepPositionConfig(cfg, target_encoder_pos);
  pos->step_cnt_prev = current_hw_cnt;
  pos->step_dcnt = 0;
}

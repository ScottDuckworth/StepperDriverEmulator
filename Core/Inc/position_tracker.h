#ifndef __POSITION_TRACKER_H
#define __POSITION_TRACKER_H

#include <stdint.h>
#include <stdbool.h>
#include "emulator_config.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  int32_t step_pos;
  int32_t encoder_pos;
  uint16_t step_cnt_prev;
  uint16_t step_dcnt;
  bool step_reverse;
} PositionCounters_t;

uint16_t CalcStepDelta(uint16_t current_hw_cnt, uint16_t prev_hw_cnt);
int32_t AccumulateStepPosition(int32_t current_step_pos, uint16_t delta, bool step_reverse);
void RealignPositionCounters(PositionCounters_t* pos, int32_t target_encoder_pos, const EmulatorConfig_t* cfg, uint16_t current_hw_cnt);

#ifdef __cplusplus
}
#endif

#endif /* __POSITION_TRACKER_H */

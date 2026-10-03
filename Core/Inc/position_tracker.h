#ifndef __POSITION_TRACKER_H
#define __POSITION_TRACKER_H

#include <stdint.h>
#include <stdbool.h>
#include "emulator_config.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  int64_t encoder_pos;        // Actual emitted encoder position
  int64_t commanded_pos;      // Commanded target position (in encoder counts)
  int32_t step_rem;           // Fractional step remainder for ratio accumulator
  uint16_t step_cnt_prev;
  uint16_t step_dcnt;
  bool step_reverse;
} PositionCounters_t;

uint16_t CalcStepDelta(uint16_t current_hw_cnt, uint16_t prev_hw_cnt);
int64_t AccumulateStepPosition(int64_t current_step_pos, uint16_t delta, bool step_reverse);
void RealignPositionCounters(PositionCounters_t* pos, int64_t target_encoder_pos, const EmulatorConfig_t* cfg, uint16_t current_hw_cnt);
uint16_t FilterStepWithBlanking(uint32_t period, uint32_t min_blanking_ticks, uint32_t* accum_ticks, uint32_t* out_valid_period);

#ifdef __cplusplus
}
#endif

#endif /* __POSITION_TRACKER_H */

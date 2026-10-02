#ifndef __MOTION_PLANNER_H
#define __MOTION_PLANNER_H

#include <stdint.h>
#include <stdbool.h>
#include "emulator_config.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  const EmulatorConfig_t* cfg;
  int64_t commanded_pos;
  int64_t planned_encoder_pos;
  int32_t load_tension;
  uint32_t now;
  uint32_t last_step_time;
  uint32_t step_period_cnt;
  bool step_reverse;
  bool is_freewheeling;
  bool stall_tripped;
  uint16_t chunk_size;
} MotionPlanRequest_t;

typedef struct {
  float target_velocity;
  int dir;
  uint16_t count_to_emit;
  bool is_freewheeling;
  bool stall_tripped;
  bool stall_trip_event;
} MotionPlanResult_t;

void PlanMotionStep(const MotionPlanRequest_t* req, MotionPlanResult_t* res);

#ifdef __cplusplus
}
#endif

#endif /* __MOTION_PLANNER_H */

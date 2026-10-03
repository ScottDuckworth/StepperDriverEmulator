#ifndef __MOTION_PLANNER_H
#define __MOTION_PLANNER_H

#include <stdint.h>
#include <stdbool.h>
#include "emulator_config.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
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

typedef struct {
  int64_t commanded_pos;
  int64_t encoder_pos;
  int64_t planned_encoder_pos;
  uint16_t step_dcnt;
  int32_t load_tension;
  uint32_t time_since_last_step_ms;
  uint32_t step_timeout_ms;
  bool is_freewheeling;
} MotionIdleCheckRequest_t;

void PlanMotionStep(const EmulatorConfig_t* cfg, const MotionPlanRequest_t* req, MotionPlanResult_t* res);
uint32_t CalcStepTimeoutMs(uint32_t step_period_cnt);
bool Motion_ShouldStart(int64_t commanded_pos, int64_t encoder_pos, uint16_t step_dcnt, int32_t load_tension, int32_t torque_t0, bool is_freewheeling);
bool Motion_ShouldStop(const EmulatorConfig_t* cfg, const MotionIdleCheckRequest_t* req);

#ifdef __cplusplus
}
#endif

#endif /* __MOTION_PLANNER_H */

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

/* Quadrature chunk buffer and state destination */
typedef struct {
  uint32_t* chunk;              /* Output DMA buffer chunk */
  uint8_t* inout_quad_state;    /* Current quadrature state (0..3) */
} MotionChunkRequest_t;

/* Emitted motion chunk outcome and timer pacing */
typedef struct {
  int8_t delta;                 /* Signed encoder count delta emitted in this chunk */
  uint16_t psc;                 /* Timer prescaler calculated for pacing */
  uint16_t arr;                 /* Timer auto-reload value calculated for pacing */
  bool stall_trip_event;        /* True if a new stall trip condition occurred */
} MotionChunkResult_t;

/* Motion start evaluation and buffer priming parameters */
typedef struct {
  int64_t commanded_pos;        /* Commanded step position in encoder counts */
  int64_t encoder_pos;          /* Current encoder position */
  int32_t load_tension;         /* External load tension on rotor */
  uint16_t step_dcnt;           /* Input step delta count */
  uint32_t now;                 /* Current system time (ms) */
  uint32_t last_step_time;      /* Timestamp of last received step pulse (ms) */
  bool step_reverse;            /* Step direction flag */
  bool is_freewheeling;         /* Motor freewheeling status */
  bool stall_tripped;           /* Motor stall latch status */
  uint16_t chunk_size;          /* Size of each DMA half-buffer chunk */
  uint32_t* quad_buffer;        /* Double-buffer base pointer (size 2 * chunk_size) */
  uint8_t* inout_quad_state;    /* In/out quadrature state accumulator */
} MotionStartRequest_t;

/* Output results from priming motion startup buffers */
typedef struct {
  uint16_t psc;                 /* Pacing prescaler for chunk 0 */
  uint16_t arr;                 /* Pacing auto-reload for chunk 0 */
  int8_t half_0_delta;          /* Encoder counts emitted in chunk 0 */
  int8_t half_1_delta;          /* Encoder counts emitted in chunk 1 */
  int64_t planned_encoder_pos;  /* Advanced planned position after priming both chunks */
  bool stall_trip_event;        /* True if stall tripped during startup priming */
} MotionStartResult_t;

void PlanMotionStep(const EmulatorConfig_t* cfg, const MotionPlanRequest_t* req, MotionPlanResult_t* res);

/*
 * Plans a single motion chunk, fills the DMA buffer with quadrature patterns,
 * and computes the required timer pacing (PSC/ARR).
 * Returns true if chunk was successfully emitted, false if arguments are invalid.
 */
bool Motion_PlanAndEmitChunk(const EmulatorConfig_t* cfg,
                            const MotionPlanRequest_t* plan_req,
                            const MotionChunkRequest_t* chunk_req,
                            MotionChunkResult_t* out_res);

/*
 * Evaluates whether motion should start, primes double-buffer chunks 0 and 1,
 * verifies non-zero motion, and prepares initial pacing registers.
 * Returns true if motion should begin and hardware should be armed; false if aborted.
 */
bool Motion_PrepareStart(const EmulatorConfig_t* cfg,
                        const MotionStartRequest_t* req,
                        MotionStartResult_t* out_res);
uint32_t CalcStepTimeoutMs(uint32_t step_period_cnt);
bool Motion_ShouldStart(int64_t commanded_pos, int64_t encoder_pos, uint16_t step_dcnt, int32_t load_tension, int32_t torque_t0, bool is_freewheeling);
bool Motion_ShouldStop(const EmulatorConfig_t* cfg, const MotionIdleCheckRequest_t* req);

#ifdef __cplusplus
}
#endif

#endif /* __MOTION_PLANNER_H */

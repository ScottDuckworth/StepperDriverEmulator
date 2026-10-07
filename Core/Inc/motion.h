#ifndef __MOTION_H
#define __MOTION_H

#include <stdint.h>
#include <stdbool.h>
#include "emulator_config.h"
#include "mathutil.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Motor pull-out torque and dynamic load calculations */
int32_t Motion_CalcMotorTorque(const EmulatorConfig_t* cfg, int32_t speed_counts_sec);

static inline int32_t Motion_CalcNetTorque(int32_t t_motor, int dir, int32_t load_tension) {
  (void) dir;
  int32_t abs_tension = MathUtil_AbsI32(load_tension);
  return t_motor - abs_tension;
}

static inline int32_t Motion_CalcFreewheelVelocity(const EmulatorConfig_t* cfg, int32_t load_tension) {
  if (!cfg) return 0;
  int32_t v_free = MathUtil_MulQ12(load_tension, cfg->persistent.kfree);
  int32_t v_max = (int32_t) cfg->cached.tcurve_max_v;
  return MathUtil_ClampI32(v_free, -v_max, v_max);
}

static inline int32_t Motion_CalcSlipVelocity(const EmulatorConfig_t* cfg, int32_t load_tension, int32_t t_motor) {
  if (!cfg) return 0;
  int32_t abs_tension = MathUtil_AbsI32(load_tension);
  if (abs_tension <= t_motor) {
    return 0;
  }
  int32_t v_slip = MathUtil_MulQ12(abs_tension - t_motor, cfg->persistent.kfree);
  int32_t v_max = (int32_t) cfg->cached.tcurve_max_v;
  if (v_slip > v_max) {
    v_slip = v_max;
  }
  return v_slip;
}

/* Motion start evaluation and state parameters */
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
} Motion_StartRequest_t;

/* Buffer pointers and parameters for priming startup double-buffers */
typedef struct {
  uint16_t chunk_size;          /* Size of each DMA half-buffer chunk */
  uint32_t* quad_buffer;        /* Double-buffer base pointer (size 2 * chunk_size) */
  uint8_t* inout_quad_state;    /* In/out quadrature state accumulator */
} Motion_StartBuffers_t;

/* Output results from priming motion startup buffers */
typedef struct {
  uint16_t psc;                 /* Pacing prescaler for chunk 0 */
  uint16_t arr;                 /* Pacing auto-reload for chunk 0 */
  int8_t half_0_delta;          /* Encoder counts emitted in chunk 0 */
  int8_t half_1_delta;          /* Encoder counts emitted in chunk 1 */
  int64_t planned_encoder_pos;  /* Advanced planned position after priming both chunks */
  bool stall_trip_event;        /* True if stall tripped during startup priming */
  int32_t target_velocity;      /* Planned output velocity in counts/sec */
} Motion_StartResult_t;

/* Motion step planning parameters */
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
} Motion_PlanStepRequest_t;

/* Planned step outcome and target velocity */
typedef struct {
  int32_t target_velocity;      /* Target velocity in encoder counts/sec */
  int dir;
  uint16_t count_to_emit;
  bool is_freewheeling;
  bool stall_tripped;
  bool stall_trip_event;
} Motion_PlanStepResult_t;

/* Quadrature chunk buffer and state destination */
typedef struct {
  uint32_t* chunk;              /* Output DMA buffer chunk */
  uint8_t* inout_quad_state;    /* Current quadrature state (0..3) */
} Motion_ChunkRequest_t;

/* Emitted motion chunk outcome and timer pacing */
typedef struct {
  int8_t delta;                 /* Signed encoder count delta emitted in this chunk */
  uint16_t psc;                 /* Timer prescaler calculated for pacing */
  uint16_t arr;                 /* Timer auto-reload value calculated for pacing */
  bool stall_trip_event;        /* True if a new stall trip condition occurred */
  int32_t target_velocity;      /* Planned output velocity in counts/sec */
} Motion_ChunkResult_t;

/* Parameters for evaluating whether motion should terminate into idle */
typedef struct {
  int64_t commanded_pos;
  int64_t encoder_pos;
  int64_t planned_encoder_pos;
  uint16_t step_dcnt;
  int32_t load_tension;
  uint32_t time_since_last_step_ms;
  uint32_t step_timeout_ms;
  bool is_freewheeling;
} Motion_StopRequest_t;

/*
 * Evaluates whether the motor should transition from idle to active motion
 * based on position error, step delta, or external load tension.
 * This predicate is evaluated first while idle; if true, Motion_PrepareStart
 * may then be called to prime double-buffers and prepare initial timer registers.
 */
bool Motion_ShouldStart(const EmulatorConfig_t* cfg, const Motion_StartRequest_t* req);

/*
 * Prepares and primes double-buffer chunks 0 and 1 prior to starting active streaming.
 * Verifies non-zero motion and prepares initial pacing registers.
 * Called after Motion_ShouldStart returns true.
 * Returns true if motion should begin and hardware should be armed; false if aborted.
 */
bool Motion_PrepareStart(const EmulatorConfig_t* cfg,
                        const Motion_StartRequest_t* req,
                        const Motion_StartBuffers_t* buf,
                        Motion_StartResult_t* out_res);

/*
 * Calculates target velocity, step direction, and pulse count to emit for a motion
 * step based on commanded position, feedback error, and velocity feedforward.
 */
void Motion_PlanStep(const EmulatorConfig_t* cfg,
                     const Motion_PlanStepRequest_t* req,
                     Motion_PlanStepResult_t* res);

/*
 * Plans a single motion chunk, fills the DMA buffer with quadrature patterns,
 * and computes the required timer pacing (PSC/ARR).
 * Returns true if chunk was successfully emitted, false if arguments are invalid.
 */
bool Motion_PlanAndEmitChunk(const EmulatorConfig_t* cfg,
                            const Motion_PlanStepRequest_t* plan_req,
                            const Motion_ChunkRequest_t* chunk_req,
                            Motion_ChunkResult_t* out_res);

/*
 * Calculates dynamic inter-step timeout in milliseconds based on measured step period.
 */
uint32_t Motion_CalcStepTimeoutMs(uint32_t step_period_cnt);

/*
 * Evaluates whether active motion should terminate and enter idle
 * based on position convergence, timeout, and load tension.
 */
bool Motion_ShouldStop(const EmulatorConfig_t* cfg, const Motion_StopRequest_t* req);

#ifdef __cplusplus
}
#endif

#endif /* __MOTION_H */

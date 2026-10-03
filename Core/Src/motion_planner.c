#include "motion_planner.h"
#include "motion_math.h"
#include "quadrature.h"
#include <math.h>
#include <stdlib.h>

void PlanMotionStep(const EmulatorConfig_t* cfg, const MotionPlanRequest_t* req, MotionPlanResult_t* res) {
  if (!cfg || !req || !res) return;

  res->target_velocity = 0.0f;
  res->dir = 0;
  res->count_to_emit = 0;
  res->is_freewheeling = req->is_freewheeling;
  res->stall_tripped = req->stall_tripped;
  res->stall_trip_event = false;

  uint16_t chunk_sz = (req->chunk_size > 0) ? req->chunk_size : 8;

  if (req->is_freewheeling) {
    float v_free = CalcFreewheelVelocity(cfg, req->load_tension);
    if (v_free != 0.0f) {
      res->dir = (v_free > 0.0f) ? 1 : -1;
      res->count_to_emit = chunk_sz;
      res->target_velocity = v_free / 1000.0f;
    } else {
      res->dir = 0;
      res->count_to_emit = 0;
      res->target_velocity = 0.0f;
    }
    return;
  }

  float step_counts = cfg->counts_per_step;
  float rate_scale = 48000.0f * step_counts;

  uint32_t step_timeout_ms = CalcStepTimeoutMs(req->step_period_cnt);

  float input_rate = 0.0f;
  if ((req->now - req->last_step_time) <= step_timeout_ms && req->step_period_cnt >= 240 && cfg->ratio_spr > 0) {
    input_rate = rate_scale / (float) req->step_period_cnt;
    if (req->step_reverse) {
      input_rate = -input_rate;
    }
  }

  int64_t d = req->commanded_pos - req->planned_encoder_pos;
  int32_t error = (d > INT32_MAX) ? INT32_MAX : ((d < INT32_MIN) ? INT32_MIN : (int32_t) d);
  float eff_error = (float) error;

  // Soft-knee error profile during active step pulse streaming:
  // When pulses are streaming (input_rate != 0), normal discrete pulse arrivals cause
  // error to fluctuate within 1 step (epr / spr counts) as the current step is being
  // paced across the inter-step interval by velocity feedforward (Kff * input_rate).
  // Deducting the feedforward-managed step window eliminates double-counting the step
  // and prevents cyclic pacing frequency modulation across the phase, while restoring
  // gain (Kp) remains active for true tracking lag (> 1 step) or overshoot (< 0).
  if (input_rate != 0.0f) {
    float ff_window = cfg->kff * step_counts;

    if (input_rate > 0.0f) {
      if (eff_error > ff_window) {
        eff_error -= ff_window;
      } else if (eff_error >= 0.0f) {
        eff_error = 0.0f;
      }
    } else {
      if (eff_error < -ff_window) {
        eff_error += ff_window;
      } else if (eff_error <= 0.0f) {
        eff_error = 0.0f;
      }
    }

    float abs_err = fabsf(eff_error);
    if (abs_err <= step_counts) {
      eff_error = (eff_error * abs_err) * cfg->inv_counts_per_step;
    }
  }

  float target_velocity = cfg->kff * input_rate + cfg->kp * eff_error;
  if (input_rate > 0.0f) {
    if (target_velocity < 0.0f) {
      target_velocity = 0.0f;
    } else {
      float max_v = fabsf(input_rate) * 1.25f + cfg->kp * step_counts;
      if (target_velocity > max_v) {
        target_velocity = max_v;
      }
    }
  } else if (input_rate < 0.0f) {
    if (target_velocity > 0.0f) {
      target_velocity = 0.0f;
    } else {
      float min_v = -(fabsf(input_rate) * 1.25f + cfg->kp * step_counts);
      if (target_velocity < min_v) {
        target_velocity = min_v;
      }
    }
  }
  res->target_velocity = target_velocity;

  if (error != 0 || input_rate != 0.0f) {
    int dir = 0;
    if (input_rate > 0.0f) {
      dir = 1;
    } else if (input_rate < 0.0f) {
      dir = -1;
    } else if (target_velocity > 0.0f) {
      dir = 1;
    } else if (target_velocity < 0.0f) {
      dir = -1;
    } else if (error != 0) {
      dir = (error > 0) ? 1 : -1;
    }

    float speed_hz = fabsf(target_velocity) * 1000.0f;
    int32_t t_motor = CalcMotorTorqueConfig(cfg, speed_hz);
    int32_t t_net = CalcNetTorque(t_motor, dir, req->load_tension);

    uint32_t abs_error = (error < 0) ? (0U - (uint32_t) error) : (uint32_t) error;

    if (t_net >= 0) {
      // Sufficient torque: motor drives normally toward target
      res->dir = dir;
      if (dir == 0) {
        res->count_to_emit = 0;
      } else if (dir > 0 && error <= 0) {
        res->count_to_emit = 0;
      } else if (dir < 0 && error >= 0) {
        res->count_to_emit = 0;
      } else {
        res->count_to_emit = (abs_error < (uint32_t) chunk_sz) ? (uint16_t) abs_error : chunk_sz;
      }
    } else {
      // Torque deficit: motor cannot advance in commanded direction
      float v_slip = CalcSlipVelocity(cfg, req->load_tension);
      if (v_slip > 0.0f) {
        res->dir = (req->load_tension > 0) ? 1 : -1;
        res->count_to_emit = chunk_sz;
        res->target_velocity = (res->dir > 0) ? (v_slip / 1000.0f) : -(v_slip / 1000.0f);
      } else {
        res->dir = 0;
        res->count_to_emit = 0;
      }

      // Under torque deficit, motor stalls and accumulates lag against commanded steps
      if (cfg->stall_threshold > 0 && abs_error >= cfg->stall_threshold) {
        if (!req->stall_tripped) {
          res->stall_trip_event = true;
        }
        res->stall_tripped = true;
        res->is_freewheeling = true;
      }
    }
  } else {
    // error == 0 and input_rate == 0: motor is at target
    float v_slip = CalcSlipVelocity(cfg, req->load_tension);
    if (v_slip > 0.0f) {
      res->dir = (req->load_tension > 0) ? 1 : -1;
      res->count_to_emit = chunk_sz;
      res->target_velocity = (res->dir > 0) ? (v_slip / 1000.0f) : -(v_slip / 1000.0f);
    } else {
      res->dir = 0;
      res->count_to_emit = 0;
    }
  }
}

uint32_t CalcStepTimeoutMs(uint32_t step_period_cnt) {
  uint32_t step_timeout_ms = 50;
  if (step_period_cnt >= 48000) {
    uint32_t period_ms = step_period_cnt / 48000;
    uint32_t dynamic_timeout = period_ms + (period_ms >> 1) + 10;
    if (dynamic_timeout > step_timeout_ms) {
      step_timeout_ms = (dynamic_timeout < 150) ? dynamic_timeout : 150;
    }
  }
  return step_timeout_ms;
}

bool Motion_ShouldStart(int64_t commanded_pos, int64_t encoder_pos, uint16_t step_dcnt, int32_t load_tension, int32_t torque_t0, bool is_freewheeling) {
  if (is_freewheeling) {
    return (load_tension != 0);
  }
  int32_t abs_tension = (load_tension >= 0) ? load_tension : -load_tension;
  return (commanded_pos != encoder_pos) || (step_dcnt != 0) || (abs_tension > torque_t0);
}

bool Motion_ShouldStop(const EmulatorConfig_t* cfg, const MotionIdleCheckRequest_t* req) {
  if (!cfg || !req) return false;
  if (req->is_freewheeling) {
    return (req->load_tension == 0);
  }
  if (req->time_since_last_step_ms < req->step_timeout_ms) {
    return false;
  }
  int32_t abs_tension = (req->load_tension >= 0) ? req->load_tension : -req->load_tension;
  return (req->commanded_pos == req->encoder_pos &&
          req->encoder_pos == req->planned_encoder_pos &&
          req->step_dcnt == 0 &&
          abs_tension <= cfg->torque_t0);
}

/*
 * Motion_PlanAndEmitChunk:
 * Evaluates the motion profile for one chunk interval, renders the corresponding
 * quadrature edge patterns into the DMA buffer slice, and computes timer pacing.
 */
bool Motion_PlanAndEmitChunk(const EmulatorConfig_t* cfg,
                            const MotionPlanRequest_t* plan_req,
                            const MotionChunkRequest_t* chunk_req,
                            MotionChunkResult_t* out_res) {
  if (!cfg || !plan_req || !chunk_req || !chunk_req->chunk || !chunk_req->inout_quad_state) {
    return false;
  }

  MotionPlanResult_t plan_res;
  PlanMotionStep(cfg, plan_req, &plan_res);

  uint16_t chunk_sz = (plan_req->chunk_size > 0) ? plan_req->chunk_size : 8;
  int8_t delta = GenerateQuadChunk(chunk_req->chunk, chunk_sz, chunk_req->inout_quad_state, plan_res.dir, plan_res.count_to_emit);

  float pace_velocity = plan_res.target_velocity;
  if (pace_velocity == 0.0f) {
    pace_velocity = cfg->counts_per_step;
  }

  uint16_t psc = 0;
  uint16_t arr = 0;
  CalcTimerPacing(pace_velocity, &psc, &arr);

  if (out_res) {
    out_res->delta = delta;
    out_res->psc = psc;
    out_res->arr = arr;
    out_res->stall_trip_event = plan_res.stall_trip_event;
  }
  return true;
}

/*
 * Motion_PrepareStart:
 * Validates motion startup conditions, primes both chunks of the circular DMA
 * buffer (chunk 0 and chunk 1), verifies non-zero movement, and returns initial pacing.
 */
bool Motion_PrepareStart(const EmulatorConfig_t* cfg,
                        const MotionStartRequest_t* req,
                        MotionStartResult_t* out_res) {
  if (!cfg || !req || !req->quad_buffer || !req->inout_quad_state) return false;

  if (!Motion_ShouldStart(req->commanded_pos, req->encoder_pos, req->step_dcnt, req->load_tension, cfg->torque_t0, req->is_freewheeling)) {
    return false;
  }

  uint16_t chunk_size = (req->chunk_size > 0) ? req->chunk_size : 8;
  int64_t planned_pos = req->encoder_pos;
  bool stall_event = false;

  // Prime Chunk 0 (startup_sync_count = 1 -> step_period_cnt = 0)
  MotionPlanRequest_t p_req = {
      .commanded_pos = req->commanded_pos,
      .planned_encoder_pos = planned_pos,
      .load_tension = req->load_tension,
      .now = req->now,
      .last_step_time = req->last_step_time,
      .step_period_cnt = 0,
      .step_reverse = req->step_reverse,
      .is_freewheeling = req->is_freewheeling,
      .stall_tripped = req->stall_tripped,
      .chunk_size = chunk_size
  };

  MotionChunkRequest_t c_req0 = {
      .chunk = &req->quad_buffer[0],
      .inout_quad_state = req->inout_quad_state
  };
  MotionChunkResult_t c_res0 = {0};
  Motion_PlanAndEmitChunk(cfg, &p_req, &c_req0, &c_res0);
  planned_pos += c_res0.delta;
  if (c_res0.stall_trip_event) stall_event = true;

  // Prime Chunk 1
  p_req.planned_encoder_pos = planned_pos;
  p_req.is_freewheeling = req->is_freewheeling || stall_event;
  p_req.stall_tripped = req->stall_tripped || stall_event;

  MotionChunkRequest_t c_req1 = {
      .chunk = &req->quad_buffer[chunk_size],
      .inout_quad_state = req->inout_quad_state
  };
  MotionChunkResult_t c_res1 = {0};
  Motion_PlanAndEmitChunk(cfg, &p_req, &c_req1, &c_res1);
  planned_pos += c_res1.delta;
  if (c_res1.stall_trip_event) stall_event = true;

  if (c_res0.delta == 0 && c_res1.delta == 0 && !Motion_ShouldStart(req->commanded_pos, req->encoder_pos, req->step_dcnt, req->load_tension, cfg->torque_t0, req->is_freewheeling)) {
    return false;
  }

  if (out_res) {
    out_res->psc = c_res0.psc;
    out_res->arr = c_res0.arr;
    out_res->half_0_delta = c_res0.delta;
    out_res->half_1_delta = c_res1.delta;
    out_res->planned_encoder_pos = planned_pos;
    out_res->stall_trip_event = stall_event;
  }
  return true;
}

#include "motion_planner.h"
#include "motion_math.h"
#include <math.h>
#include <stdlib.h>

void PlanMotionStep(const MotionPlanRequest_t* req, MotionPlanResult_t* res) {
  if (!req || !res) return;

  res->target_velocity = 0.0f;
  res->dir = 0;
  res->count_to_emit = 0;
  res->is_freewheeling = req->is_freewheeling;
  res->stall_tripped = req->stall_tripped;
  res->stall_trip_event = false;

  if (!req->cfg) return;

  uint16_t chunk_sz = (req->chunk_size > 0) ? req->chunk_size : 8;

  if (req->is_freewheeling) {
    float v_free = CalcFreewheelVelocity(req->cfg, req->load_tension);
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

  float input_rate = 0.0f;
  if ((req->now - req->last_step_time) <= 50 && req->step_period_cnt > 0 && req->cfg->spr > 0) {
    input_rate = (48000.0f / (float) req->step_period_cnt) * ((float) req->cfg->epr / (float) req->cfg->spr);
    if (req->step_reverse) {
      input_rate = -input_rate;
    }
  }

  int32_t error = req->commanded_pos - req->planned_encoder_pos;
  float target_velocity = req->cfg->kff * input_rate + req->cfg->kp * (float) error;
  res->target_velocity = target_velocity;

  if (error != 0) {
    int dir = (error > 0) ? 1 : -1;
    float speed_hz = fabsf(target_velocity) * 1000.0f;
    int32_t t_motor = CalcMotorTorqueConfig(req->cfg, speed_hz);
    int32_t t_net = CalcNetTorque(t_motor, dir, req->load_tension);

    if (t_net >= 0) {
      // Sufficient torque: motor drives normally toward target
      int32_t abs_error = (error > 0) ? error : -error;
      res->dir = dir;
      res->count_to_emit = (abs_error < (int32_t) chunk_sz) ? (uint16_t) abs_error : chunk_sz;
    } else {
      // Torque deficit: motor cannot advance in commanded direction
      float v_slip = CalcSlipVelocity(req->cfg, req->load_tension);
      if (v_slip > 0.0f) {
        res->dir = (req->load_tension > 0) ? 1 : -1;
        res->count_to_emit = chunk_sz;
        res->target_velocity = (res->dir > 0) ? (v_slip / 1000.0f) : -(v_slip / 1000.0f);
      } else {
        res->dir = 0;
        res->count_to_emit = 0;
      }

      // Under torque deficit, motor stalls and accumulates lag against commanded steps
      uint32_t lag = (error >= 0) ? (uint32_t) error : (uint32_t)(-error);
      if (req->cfg->stall_threshold > 0 && lag >= req->cfg->stall_threshold) {
        if (!req->stall_tripped) {
          res->stall_trip_event = true;
        }
        res->stall_tripped = true;
        res->is_freewheeling = true;
      }
    }
  } else {
    // error == 0: motor is at target
    float v_slip = CalcSlipVelocity(req->cfg, req->load_tension);
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

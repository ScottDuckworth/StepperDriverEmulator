#include "motion_math.h"
#include <stdint.h>

uint16_t CalcGCD(uint16_t a, uint16_t b) {
  while (b != 0) {
    uint16_t t = b;
    b = a % b;
    a = t;
  }
  return a;
}

int32_t ConvertStepDeltaToCounts(int32_t step_delta, uint16_t ratio_spr, uint16_t ratio_epr, int32_t* remainder) {
  if (ratio_spr == 0) return 0;
  int32_t rem = remainder ? *remainder : 0;
  int32_t accum = rem + step_delta * (int32_t) ratio_epr;
  int32_t counts = accum / (int32_t) ratio_spr;
  rem = accum % (int32_t) ratio_spr;
  if (remainder) {
    *remainder = rem;
  }
  return counts;
}

int32_t CalcMotorTorqueConfig(const EmulatorConfig_t* cfg, float speed_abs) {
  if (!cfg) return 0;
  uint32_t v = (uint32_t) speed_abs;
  if (v <= cfg->torque_v_knee) {
    return cfg->torque_t0;
  }
  if (v >= cfg->torque_v_max || cfg->torque_v_max <= cfg->torque_v_knee) {
    return cfg->torque_t_min;
  }
  int32_t num = (cfg->torque_t0 - cfg->torque_t_min) * (int32_t)(v - cfg->torque_v_knee);
  int32_t den = (int32_t)(cfg->torque_v_max - cfg->torque_v_knee);
  return (den > 0) ? (cfg->torque_t0 - (num / den)) : cfg->torque_t_min;
}

int32_t CalcNetTorque(int32_t t_motor, int dir, int32_t load_tension) {
  (void) dir;
  int32_t abs_tension = (load_tension >= 0) ? load_tension : -load_tension;
  return t_motor - abs_tension;
}

float CalcFreewheelVelocity(const EmulatorConfig_t* cfg, int32_t load_tension) {
  if (!cfg) return 0.0f;
  float v_free = (float) load_tension * cfg->kfree;
  float v_max = (float) cfg->torque_v_max;
  if (v_free > v_max) {
    v_free = v_max;
  } else if (v_free < -v_max) {
    v_free = -v_max;
  }
  return v_free;
}

float CalcSlipVelocity(const EmulatorConfig_t* cfg, int32_t load_tension, int32_t t_motor) {
  if (!cfg) return 0.0f;
  int32_t abs_tension = (load_tension >= 0) ? load_tension : -load_tension;
  if (abs_tension <= t_motor) {
    return 0.0f;
  }
  float v_slip = (float)(abs_tension - t_motor) * cfg->kfree;
  float v_max = (float) cfg->torque_v_max;
  if (v_slip > v_max) {
    v_slip = v_max;
  }
  return v_slip;
}

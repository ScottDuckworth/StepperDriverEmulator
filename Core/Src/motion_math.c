#include "motion_math.h"
#include <stdint.h>

int64_t StepToEncoderPositionConfig(const EmulatorConfig_t* cfg, int64_t step_position) {
  if (!cfg || cfg->spr == 0) return 0;
  return (step_position * (int64_t) cfg->epr) / (int64_t) cfg->spr;
}

int64_t EncoderToStepPositionConfig(const EmulatorConfig_t* cfg, int64_t encoder_position) {
  if (!cfg || cfg->epr == 0) return 0;
  return (encoder_position * (int64_t) cfg->spr) / (int64_t) cfg->epr;
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
  int64_t num = (int64_t)(cfg->torque_t0 - cfg->torque_t_min) * (v - cfg->torque_v_knee);
  int64_t den = (int64_t)(cfg->torque_v_max - cfg->torque_v_knee);
  return (int32_t)(cfg->torque_t0 - (num / den));
}

int32_t CalcNetTorque(int32_t t_motor, int dir, int32_t load_tension) {
  return t_motor + dir * load_tension;
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

float CalcSlipVelocity(const EmulatorConfig_t* cfg, int32_t load_tension) {
  if (!cfg) return 0.0f;
  int32_t abs_tension = (load_tension >= 0) ? load_tension : -load_tension;
  if (abs_tension <= cfg->torque_t0) {
    return 0.0f;
  }
  float v_slip = (float)(abs_tension - cfg->torque_t0) * cfg->kfree;
  float v_max = (float) cfg->torque_v_max;
  if (v_slip > v_max) {
    v_slip = v_max;
  }
  return v_slip;
}

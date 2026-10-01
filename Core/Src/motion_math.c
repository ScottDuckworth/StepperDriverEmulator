#include "motion_math.h"
#include <stdint.h>

int32_t StepToEncoderPositionConfig(const EmulatorConfig_t* cfg, int32_t step_position) {
  if (!cfg || cfg->spr == 0) return 0;
  return (int32_t)(((int64_t) step_position * cfg->epr) / cfg->spr);
}

int32_t EncoderToStepPositionConfig(const EmulatorConfig_t* cfg, int32_t encoder_position) {
  if (!cfg || cfg->epr == 0) return 0;
  return (int32_t)(((int64_t) encoder_position * cfg->spr) / cfg->epr);
}

int32_t CalcMotorTorqueConfig(const EmulatorConfig_t* cfg, float speed_abs) {
  if (!cfg) return 0;
  uint32_t v = (uint32_t) speed_abs;
  if (v <= cfg->torque_v_knee) {
    return cfg->torque_t0;
  }
  if (v >= cfg->torque_v_max) {
    return cfg->torque_t_min;
  }
  int64_t num = (int64_t)(cfg->torque_t0 - cfg->torque_t_min) * (v - cfg->torque_v_knee);
  int64_t den = (int64_t)(cfg->torque_v_max - cfg->torque_v_knee);
  return (int32_t)(cfg->torque_t0 - (num / den));
}

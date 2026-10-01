#ifndef __MOTION_MATH_H
#define __MOTION_MATH_H

#include "emulator_config.h"

#ifdef __cplusplus
extern "C" {
#endif

int32_t StepToEncoderPositionConfig(const EmulatorConfig_t* cfg, int32_t step_position);
int32_t EncoderToStepPositionConfig(const EmulatorConfig_t* cfg, int32_t encoder_position);
int32_t CalcMotorTorqueConfig(const EmulatorConfig_t* cfg, float speed_abs);
int32_t CalcNetTorque(int32_t t_motor, int dir, int32_t load_tension);
float CalcFreewheelVelocity(const EmulatorConfig_t* cfg, int32_t load_tension);
float CalcSlipVelocity(const EmulatorConfig_t* cfg, int32_t load_tension);

#ifdef __cplusplus
}
#endif

#endif /* __MOTION_MATH_H */

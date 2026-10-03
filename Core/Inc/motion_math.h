#ifndef __MOTION_MATH_H
#define __MOTION_MATH_H

#include <stdint.h>
#include "emulator_config.h"

#ifdef __cplusplus
extern "C" {
#endif

uint16_t CalcGCD(uint16_t a, uint16_t b);
int32_t ConvertStepDeltaToCounts(int32_t step_delta, uint16_t ratio_spr, uint16_t ratio_epr, int32_t* remainder);
int32_t CalcMotorTorqueConfig(const EmulatorConfig_t* cfg, float speed_abs);
int32_t CalcNetTorque(int32_t t_motor, int dir, int32_t load_tension);
float CalcFreewheelVelocity(const EmulatorConfig_t* cfg, int32_t load_tension);
float CalcSlipVelocity(const EmulatorConfig_t* cfg, int32_t load_tension, int32_t t_motor);

#ifdef __cplusplus
}
#endif

#endif /* __MOTION_MATH_H */

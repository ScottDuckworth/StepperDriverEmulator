#ifndef __EMULATOR_CONFIG_H
#define __EMULATOR_CONFIG_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  // Timing and Resolution
  uint16_t odr;               // Output Data Rate for periodic position reports (ms, 0 = disabled)
  uint16_t ratio_spr;         // Canonical step ratio (reduced by GCD)
  uint16_t ratio_epr;         // Canonical encoder count ratio (reduced by GCD)

  // Torque-Speed Curve Parameters
  int32_t torque_t0;          // Maximum holding torque shelf below v_knee
  uint32_t torque_v_knee;     // Knee velocity where torque starts derating (encoder counts/s)
  uint32_t torque_v_max;      // High-speed cutoff where torque reaches t_min (encoder counts/s)
  int32_t torque_t_min;       // Minimum pull-out torque at and above v_max
  uint32_t stall_threshold;   // Rotor lag error threshold before tripping stall fault (encoder counts, 0 = disabled)

  // Floating-point Control Gains & Physical Coefficients
  float kp;                   // Proportional gain for position tracking error
  float kff;                  // Feedforward velocity gain for input pulse rate
  float kfree;                // Viscous freewheeling velocity coefficient under load tension

  // Pre-calculated Kinematic Conversions
  float counts_per_step;      // Pre-calculated ratio: (float) ratio_epr / (float) ratio_spr
} EmulatorConfig_t;

#define DEFAULT_EMULATOR_CONFIG { \
  .odr = 1000, \
  .ratio_spr = 1, \
  .ratio_epr = 4, \
  .torque_t0 = 1000, \
  .torque_v_knee = 1000, \
  .torque_v_max = 8000, \
  .torque_t_min = 200, \
  .stall_threshold = 4000, \
  .kp = 0.1f, \
  .kff = 1.0f, \
  .kfree = 0.005f, \
  .counts_per_step = 4.0f, \
}

#ifdef __cplusplus
}
#endif

#endif /* __EMULATOR_CONFIG_H */

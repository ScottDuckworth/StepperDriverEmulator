#ifndef __EMULATOR_CONFIG_H
#define __EMULATOR_CONFIG_H

#include <stdint.h>
#include <stdbool.h>
#include "mathutil.h"

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

  // Control Gains & Physical Coefficients (Q12 Fixed-Point)
  q12_t kp_q12;                  // Proportional gain for position tracking error (kp * 4096)
  q12_t kff_q12;                 // Feedforward velocity gain for input pulse rate (kff * 4096)
  q12_t kfree_q12;               // Viscous freewheeling velocity coefficient under load tension (kfree * 4096)

  // Cached Kinematic & Torque Parameters (Precomputed for real-time ISR)
  q12_t kp_velocity_q12;         // kp velocity gain in counts/sec (kp * 1000 * 4096)
  q12_t counts_per_step_q12;     // (ratio_epr / ratio_spr) in Q12 format
  q16_t inv_counts_per_step_q16; // (ratio_spr / ratio_epr) in Q16 format
  q16_t inv_torque_span_v_q16;   // 1 / (torque_v_max - torque_v_knee) in Q16 format
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
  .kp_q12 = { .raw = 410 }, \
  .kff_q12 = { .raw = 4096 }, \
  .kfree_q12 = { .raw = 20 }, \
  .kp_velocity_q12 = { .raw = 409600 }, \
  .counts_per_step_q12 = { .raw = 16384 }, \
  .inv_counts_per_step_q16 = { .raw = 16384 }, \
  .inv_torque_span_v_q16 = { .raw = 9 }, \
}

#ifdef __cplusplus
}
#endif

#endif /* __EMULATOR_CONFIG_H */

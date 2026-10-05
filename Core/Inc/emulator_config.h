#ifndef __EMULATOR_CONFIG_H
#define __EMULATOR_CONFIG_H

#include <stdint.h>
#include <stdbool.h>
#include "mathutil.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CONTROLLER_NAME_MAX_LEN 32

typedef struct {
  // Controller Identification
  char name[CONTROLLER_NAME_MAX_LEN]; // Controller identifier string (null-terminated, up to 31 chars)

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

  // Control Gains & Physical Coefficients
  q12_t kp;                  // Proportional gain for position tracking error (kp * 4096)
  q12_t kff;                 // Feedforward velocity gain for input pulse rate (kff * 4096)
  q12_t kfree;               // Viscous freewheeling velocity coefficient under load tension (kfree * 4096)
} PersistentConfig_t;

typedef struct {
  // Cached Kinematic & Torque Parameters (Precomputed for real-time ISR)
  q12_t kp_velocity;         // kp velocity gain in counts/sec (kp * 1000 * 4096)
  q12_t counts_per_step;     // (ratio_epr / ratio_spr)
  q16_t inv_counts_per_step; // (ratio_spr / ratio_epr)
  q16_t inv_torque_span_v;   // 1 / (torque_v_max - torque_v_knee)
  q16_t torque_derate_slope; // (torque_t0 - torque_t_min) / (torque_v_max - torque_v_knee)
  int32_t step_counts_int;   // Integer counts per step (MathUtil_Q12ToInt(counts_per_step))
  int32_t ff_window;         // Feedforward deadband window (MathUtil_Q12ToInt(kff * counts_per_step))
  int32_t max_kp_step_v;     // Maximum velocity offset for 1 step lag (step_counts_int * kp_velocity)
  uint32_t clock_counts_sec; // Timer tick scaling in counts/sec: (48000000 * ratio_epr) / ratio_spr (0 if ratio > 89)
} CachedConfig_t;

typedef struct {
  PersistentConfig_t persistent;
  CachedConfig_t cached;
} EmulatorConfig_t;

#define DEFAULT_PERSISTENT_CONFIG { \
  .name = {0}, \
  .odr = 1000, \
  .ratio_spr = 1, \
  .ratio_epr = 4, \
  .torque_t0 = 1000, \
  .torque_v_knee = 1000, \
  .torque_v_max = 8000, \
  .torque_t_min = 200, \
  .stall_threshold = 4000, \
  .kp = { .raw = 410 }, \
  .kff = { .raw = 4096 }, \
  .kfree = { .raw = 20 }, \
}

#define DEFAULT_EMULATOR_CONFIG { \
  .persistent = DEFAULT_PERSISTENT_CONFIG, \
  .cached = {{0}}, \
}

#ifdef __cplusplus
}
#endif

#endif /* __EMULATOR_CONFIG_H */

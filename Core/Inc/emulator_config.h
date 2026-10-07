#ifndef __EMULATOR_CONFIG_H
#define __EMULATOR_CONFIG_H

#include <stdint.h>
#include <stdbool.h>
#include "mathutil.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CONTROLLER_NAME_MAX_LEN 32
#define TCURVE_MAX_POINTS 33

typedef struct {
  // Controller Identification
  char name[CONTROLLER_NAME_MAX_LEN]; // Controller identifier string (null-terminated, up to 31 chars)

  // Timing and Resolution
  uint16_t odr;               // Output Data Rate for periodic position reports (ms, 0 = disabled)
  uint16_t ratio_spr;         // Canonical step ratio (reduced by GCD)
  uint16_t ratio_epr;         // Canonical encoder count ratio (reduced by GCD)

  // Torque-Speed Lookup Table Parameters
  uint32_t tcurve_delta_v;                    // Uniform velocity step size (encoder counts/s)
  uint8_t tcurve_point_count;                 // Active knot count (2 to 33)
  int32_t tcurve_table[TCURVE_MAX_POINTS];    // Torque values from v=0 to v_max = (point_count - 1) * delta_v
  uint32_t stall_threshold;   // Rotor lag error threshold before tripping stall fault (encoder counts, 0 = disabled)
  int64_t minstop;            // Lower physical travel stop (encoder counts, INT64_MIN = cleared)
  int64_t maxstop;            // Upper physical travel stop (encoder counts, INT64_MAX = cleared)

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
  uint32_t tcurve_max_v;     // (tcurve_point_count - 1) * tcurve_delta_v
  q16_t inv_tcurve_delta_v;  // Q16 fixed-point reciprocal (65536 / tcurve_delta_v)
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
  .tcurve_delta_v = 250, \
  .tcurve_point_count = 33, \
  .tcurve_table = { \
    1000, 1000, 1000, 1000, 1000, 971, 943, 914, 886, 857, 829, 800, 771, 743, 714, 686, \
    657, 629, 600, 571, 543, 514, 486, 457, 429, 400, 371, 343, 314, 286, 257, 229, 200 \
  }, \
  .stall_threshold = 4000, \
  .minstop = INT64_MIN, \
  .maxstop = INT64_MAX, \
  .kp = Q12_INIT_RATIO(1, 10), \
  .kff = Q12_INIT_INT(1), \
  .kfree = Q12_INIT_RATIO(1, 200), \
}

#define DEFAULT_EMULATOR_CONFIG { \
  .persistent = DEFAULT_PERSISTENT_CONFIG, \
  .cached = {{0}}, \
}

#ifdef __cplusplus
}
#endif

#endif /* __EMULATOR_CONFIG_H */

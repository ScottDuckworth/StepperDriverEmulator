#include "led_control.h"

void EvalLEDState(uint32_t now, bool blink_mode, bool stall_tripped, bool step_enabled, uint32_t last_step_time, uint8_t* out_r, uint8_t* out_g) {
  bool blink_phase = ((now / 125) & 1) != 0;
  uint8_t r = 0;
  uint8_t g = 0;

  if (blink_mode) {
    r = blink_phase ? 255 : 0;
    g = blink_phase ? 255 : 0;
  } else if (stall_tripped) {
    r = blink_phase ? 255 : 0;
    g = 0;
  } else if (!step_enabled) {
    r = 0;
    g = 0;
  } else if ((now - last_step_time) < 200) {
    r = 0;
    g = blink_phase ? 255 : 0;
  } else {
    r = 0;
    g = 255;
  }

  if (out_r) *out_r = r;
  if (out_g) *out_g = g;
}

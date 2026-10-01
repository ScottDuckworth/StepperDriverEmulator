#ifndef __LED_CONTROL_H
#define __LED_CONTROL_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void EvalLEDState(uint32_t now, bool blink_mode, bool stall_tripped, bool step_enabled, uint32_t last_step_time, uint8_t* out_r, uint8_t* out_g);

#ifdef __cplusplus
}
#endif

#endif /* __LED_CONTROL_H */

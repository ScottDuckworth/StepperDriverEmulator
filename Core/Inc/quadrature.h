#ifndef __QUADRATURE_H
#define __QUADRATURE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef GPIO_BSRR_BS_4
#define GPIO_BSRR_BS_4 (1U << 4)
#define GPIO_BSRR_BR_4 (1U << 20)
#define GPIO_BSRR_BS_5 (1U << 5)
#define GPIO_BSRR_BR_5 (1U << 21)
#endif

uint8_t NextQuadState(uint8_t current_state, int dir);
uint32_t GetQuadBsrrValue(uint8_t quad_state);
int8_t GenerateQuadChunk(uint32_t* chunk, uint16_t chunk_size, uint8_t* inout_state, int dir, uint16_t count_to_emit);
void CalcTimerPacing(float target_velocity_counts_per_ms, uint16_t* out_psc, uint16_t* out_arr);

#ifdef __cplusplus
}
#endif

#endif /* __QUADRATURE_H */


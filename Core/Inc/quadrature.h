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

/*
 * Computes the next 2-bit Gray code state given the current state and direction.
 * Direction: positive (+1) advances forward, negative (-1) advances reverse, 0 remains unchanged.
 */
uint8_t Quadrature_NextState(uint8_t current_state, int dir);

/*
 * Returns the atomic 32-bit GPIO BSRR set/reset bitmask for PB4 (EA) and PB5 (EB)
 * corresponding to a 2-bit quadrature state (0-3).
 */
uint32_t Quadrature_GetBsrrValue(uint8_t quad_state);

/*
 * Synthesizes an array of atomic GPIO BSRR masks for a DMA half-buffer chunk
 * and updates the running quadrature state. Returns the signed count of transitions emitted.
 */
int8_t Quadrature_GenerateChunk(uint32_t* chunk, uint16_t chunk_size, uint8_t* inout_state, int dir, uint16_t count_to_emit);

/*
 * Computes the dynamic pacing timer prescaler (PSC) and auto-reload (ARR)
 * values for TIM3 to output quadrature steps at the target velocity in counts/sec.
 */
void Quadrature_CalcTimerPacing(uint32_t target_velocity_counts_sec, uint16_t* out_psc, uint16_t* out_arr);

#ifdef __cplusplus
}
#endif

#endif /* __QUADRATURE_H */


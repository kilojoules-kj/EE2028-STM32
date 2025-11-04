/*
 * buzzer.h
 *
 *  Created on: Nov 5, 2025
 *      Author: KJ
 */

#ifndef SRC_BUZZER_H_
#define SRC_BUZZER_H_

#include "stm32l4xx_hal.h"  // or your MCU's HAL header

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    BUZZ_NONE = 0,
    BUZZ_GAME_OVER,     // long-long-long
    BUZZ_CHANGE_ROLE,   // short-short
    BUZZ_CHANGE_GAME,   // short-long
    BUZZ_CAPTURED,      // short x3
    BUZZ_ESCAPED        // short-short-long
} BuzzerPattern;

/**
 * Call once after GPIO clocks/pins are configured.
 * The pin should already be set as push-pull output.
 */
void Buzzer_Init(GPIO_TypeDef *port, uint16_t pin);

/**
 * Start a non-blocking pattern (ISR-safe).
 * Pass HAL_GetTick() for now_ms.
 */
void Buzzer_PlayPattern(BuzzerPattern pat, uint32_t now_ms);

/** Stop any current pattern and drive pin low (ISR-safe). */
void Buzzer_Stop(void);

/**
 * Pump the pattern player frequently from the main loop
 * (e.g., every iteration). Pass HAL_GetTick().
 */
void Buzzer_Update(uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* SRC_BUZZER_H_ */

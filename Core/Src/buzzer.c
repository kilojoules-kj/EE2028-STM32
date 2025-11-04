/*
 * buzzer.c
 *
 *  Created on: Nov 5, 2025
 *      Author: KJ
 */

#include "buzzer.h"
#include <stddef.h>

// ---------- Patterns (ON,OFF,ON,OFF..., starting with ON), in ms ----------
static const uint16_t PAT_GAME_OVER[]   = { 300,150, 300,150, 300 };
static const uint16_t PAT_CHANGE_ROLE[] = { 100,120, 100 };
static const uint16_t PAT_CHANGE_GAME[] = { 100,120, 250 };
static const uint16_t PAT_CAPTURED[]    = {  80, 80,  80, 80,  80 };
static const uint16_t PAT_ESCAPED[]     = {  80,120,  80,150, 250 };

typedef struct {
    const uint16_t *seq;
    uint8_t         len;
} BuzzSeq;

static const BuzzSeq g_buzzTable[] = {
    /* BUZZ_NONE         */ { NULL, 0 },
    /* BUZZ_GAME_OVER    */ { PAT_GAME_OVER,   sizeof(PAT_GAME_OVER)   / sizeof(PAT_GAME_OVER[0]) },
    /* BUZZ_CHANGE_ROLE  */ { PAT_CHANGE_ROLE, sizeof(PAT_CHANGE_ROLE) / sizeof(PAT_CHANGE_ROLE[0]) },
    /* BUZZ_CHANGE_GAME  */ { PAT_CHANGE_GAME, sizeof(PAT_CHANGE_GAME) / sizeof(PAT_CHANGE_GAME[0]) },
    /* BUZZ_CAPTURED     */ { PAT_CAPTURED,    sizeof(PAT_CAPTURED)    / sizeof(PAT_CAPTURED[0]) },
    /* BUZZ_ESCAPED      */ { PAT_ESCAPED,     sizeof(PAT_ESCAPED)     / sizeof(PAT_ESCAPED[0]) },
};

// ---------- State ----------
static GPIO_TypeDef  *g_port = NULL;
static uint16_t       g_pin  = 0;

static volatile const uint16_t *g_seq = NULL;
static volatile uint8_t         g_len = 0;
static volatile uint8_t         g_idx = 0;
static volatile uint32_t        g_next_ms = 0;
static volatile uint8_t         g_active = 0;  // 0/1

static inline void buzzer_gpio_set(uint8_t on)
{
    if (!g_port) return;
    HAL_GPIO_WritePin(g_port, g_pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Buzzer_Init(GPIO_TypeDef *port, uint16_t pin)
{
    g_port = port;
    g_pin  = pin;
    g_seq = NULL; g_len = 0; g_idx = 0; g_next_ms = 0; g_active = 0;
    buzzer_gpio_set(0);
}

void Buzzer_Stop(void)
{
    g_active = 0;
    g_seq = NULL; g_len = 0; g_idx = 0;
    buzzer_gpio_set(0);
}

void Buzzer_PlayPattern(BuzzerPattern pat, uint32_t now_ms)
{
    if (pat <= BUZZ_NONE || pat >= (BuzzerPattern)(sizeof(g_buzzTable)/sizeof(g_buzzTable[0]))) {
        Buzzer_Stop();
        return;
    }
    const BuzzSeq *s = &g_buzzTable[pat];
    if (!s->seq || s->len == 0) { Buzzer_Stop(); return; }

    g_seq = s->seq;
    g_len = s->len;
    g_idx = 0;
    g_active = 1;

    // Start ON and schedule next toggle
    buzzer_gpio_set(1);
    g_next_ms = now_ms + g_seq[g_idx++];
    if (g_idx >= g_len) { g_active = 0; } // single element pattern
}

void Buzzer_Update(uint32_t now_ms)
{
    if (!g_active) return;
    if ((int32_t)(now_ms - g_next_ms) < 0) return;

    // Toggle
    GPIO_PinState cur = HAL_GPIO_ReadPin(g_port, g_pin);
    buzzer_gpio_set(cur == GPIO_PIN_RESET ? 1 : 0);

    // Next segment or finish
    if (g_idx < g_len) {
        g_next_ms = now_ms + g_seq[g_idx++];
    } else {
        Buzzer_Stop();
    }
}

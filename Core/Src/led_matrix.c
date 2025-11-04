/*
 * led_matrix.c
 *
 *  Created on: Nov 4, 2025
 *      Author: KJ
 */

#include "led_matrix.h"
#include <stdint.h>
#include <stdbool.h>

typedef uint8_t LedFrame[8];

static const LedFrame DIGIT_0_RIGHT = {
    0b00001111,
	0b00001001,
	0b00001001,
	0b00001001,
    0b00001001,
	0b00001001,
	0b00001111,
	0b00000000, };

static const LedFrame DIGIT_1_RIGHT = {
    0b00000001,
	0b00000001,
	0b00000001,
	0b00000001,
    0b00000001,
	0b00000001,
	0b00000001,
	0b00000000, };

static const LedFrame DIGIT_2_RIGHT = {
    0b00001111,
	0b00000001,
	0b00000001,
	0b00001111,
    0b00001000,
	0b00001000,
	0b00001111,
	0b00000000, };

static const LedFrame DIGIT_3_RIGHT = {
    0b00001111,
	0b00000001,
	0b00000001,
	0b00001111,
    0b00000001,
	0b00000001,
	0b00001111,
	0b00000000, };

static const LedFrame DIGIT_4_RIGHT = {
    0b00001001,
	0b00001001,
	0b00001001,
	0b00001111,
    0b00000001,
	0b00000001,
	0b00000001,
	0b00000000, };

static const LedFrame DIGIT_5_RIGHT = {
    0b00001111,
	0b00001000,
	0b00001000,
	0b00001111,
    0b00000001,
	0b00000001,
	0b00001111,
	0b00000000, };

static const LedFrame DIGIT_6_RIGHT = {
    0b00001111,
	0b00001000,
	0b00001000,
	0b00001111,
    0b00001001,
	0b00001001,
	0b00001111,
	0b00000000, };

static const LedFrame DIGIT_7_RIGHT = {
    0b00001111,
	0b00000001,
	0b00000001,
	0b00000001,
    0b00000001,
	0b00000001,
	0b00000001,
	0b00000000, };

static const LedFrame DIGIT_8_RIGHT = {
    0b00001111,
	0b00001001,
	0b00001001,
	0b00001111,
    0b00001001,
	0b00001001,
	0b00001111,
	0b00000000, };

static const LedFrame DIGIT_9_RIGHT = {
    0b00001111,
	0b00001001,
	0b00001001,
	0b00001111,
    0b00000001,
	0b00000001,
	0b00000001,
	0b00000000, };


static const LedFrame GREEN_LIGHT = {
    0b11100000,
	0b10100000,
	0b11100000,
	0b00000000,
    0b00000000,
	0b00000000,
	0b00000000,
	0b00000000, };

static const LedFrame RED_LIGHT = {
    0b00000000,
	0b00000000,
	0b00000000,
	0b00000000,
    0b10100000,
	0b01000000,
	0b10100000,
	0b00000000, };

// Map digits to the right-side glyphs (just pointers to your existing const tables)
static const LedFrame *const DIGITS[10] = {
    &DIGIT_0_RIGHT, &DIGIT_1_RIGHT, &DIGIT_2_RIGHT, &DIGIT_3_RIGHT, &DIGIT_4_RIGHT,
    &DIGIT_5_RIGHT, &DIGIT_6_RIGHT, &DIGIT_7_RIGHT, &DIGIT_8_RIGHT, &DIGIT_9_RIGHT
};



// tiny helpers (kept local so you don’t need a header)
static inline int clamp_digit(int d) { return (d < 0) ? 0 : (d > 9 ? 9 : d); }
static inline void copy_frame(LedFrame dst, const LedFrame src) {
    for (int i = 0; i < 8; ++i) dst[i] = src[i];
}
static inline void or_frame(LedFrame dst, const LedFrame mask) {
    for (int i = 0; i < 8; ++i) dst[i] |= mask[i];
}

/*
 * @param number, must be 0-9, -1 is clear
 * @param shape, what is the left shape (green light red light), LED_SHAPE_NONE is clear
 */
const uint8_t *led_matrix_frame(int number, int shape)
{
    static LedFrame out;

    // Special case: blank canvas request
    if (number < 0) {
        // Either memset(out, 0, sizeof out);
        for (int i = 0; i < 8; ++i) out[i] = 0;
        return out;
    }

    // base digit
    const LedFrame *digit = DIGITS[clamp_digit(number)];
    copy_frame(out, *digit);

    // optional left-side shape
    if (shape == LED_SHAPE_GREEN) {
        or_frame(out, GREEN_LIGHT);
    } else if (shape == LED_SHAPE_RED) {
        or_frame(out, RED_LIGHT);
    }

    return out;
}

#ifndef LED_MATRIX_H
#define LED_MATRIX_H

#include <stdint.h>

// 8 rows, one byte per row
typedef uint8_t LedFrame[8];

// Shapes for the left side
enum {
    LED_SHAPE_NONE  = 0,
    LED_SHAPE_GREEN = 1,
    LED_SHAPE_RED   = 2,
};

// Build a frame (left shape OR'ed with right-side digit) and
// return a pointer to an internal 8-byte buffer suitable for HT16K33_WriteRows8.
const uint8_t *led_matrix_frame(int number, int shape);

#endif // LED_MATRIX_H

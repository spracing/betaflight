#pragma once

#include "osd/font_max7456_12x18.h"

void simple_artificial_horizon(int16_t roll, int16_t pitch, int16_t x, int16_t y,
        int16_t width, int16_t height, int8_t max_pitch, uint8_t n_pitch_steps);


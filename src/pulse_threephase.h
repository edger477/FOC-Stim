#ifndef FOCSTIM_PULSE_THREEPHASE_H
#define FOCSTIM_PULSE_THREEPHASE_H

#include <math.h>
#include <cstdint>

#include "config.h"
#include "utils.h"

class ThreephasePulse
{
public:
    ThreephasePulse() = default;

    void create_pulse(
        float current_amplitude,
        float alpha,
        float beta,
        float carrier_frequency,
        float pulse_width,
        float pulse_rise,
        float center_calibration,
        float up_down_calibration,
        float left_right_calibration);

    // inline for speed
    void get(float t, float *a_out, float *b_out)
    {
        float integral, remainder;
        remainder = modff((t / pulse_duration) * THREEPHASE_PULSE_BUFFER_SIZE, &integral);
        uint32_t i = integral;
        if ((i + 1) >= THREEPHASE_PULSE_BUFFER_SIZE || t < 0)
        {
            *a_out = 0;
            *b_out = 0;
        }
        else
        {
            *a_out = lerp(remainder, a[i], a[i + 1]);
            *b_out = lerp(remainder, b[i], b[i + 1]);
        }
    }

    void print();

    float pulse_duration = 0;
    float a[THREEPHASE_PULSE_BUFFER_SIZE];
    float b[THREEPHASE_PULSE_BUFFER_SIZE];
};

#endif // FOCSTIM_PULSE_THREEPHASE_H
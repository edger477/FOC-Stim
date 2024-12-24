#ifndef FOCSTIM_THREEPHASE_PULSE_BUFFER_H
#define FOCSTIM_THREEPHASE_PULSE_BUFFER_H

#include <math.h>
#include <cstdint>

#include "config.h"
#include "utils.h"

class ThreephasePulseBuffer
{
public:
    ThreephasePulseBuffer() = default;

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
    void get(float t, float *neutral, float *left)
    {
        float integral, remainder;
        remainder = modff((t / pulse_duration) * THREEPHASE_PULSE_BUFFER_SIZE, &integral);
        uint32_t i = integral;
        if ((i + 1) >= THREEPHASE_PULSE_BUFFER_SIZE || t < 0)
        {
            *neutral = 0;
            *left = 0;
        }
        else
        {
            *neutral = a[i] + remainder * (a[i + 1] - a[i]);
            *left = b[i] + remainder * (b[i + 1] - b[i]);
        }
    }

    void print();

    float pulse_duration = 0;               // seconds
    float a[THREEPHASE_PULSE_BUFFER_SIZE];  // neutral electrode
    float b[THREEPHASE_PULSE_BUFFER_SIZE];  // left electrode
};

#endif // FOCSTIM_THREEPHASE_PULSE_BUFFER_H
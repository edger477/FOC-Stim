#ifndef STIM_CLOCK_H
#define STIM_CLOCK_H

#include <Arduino.h>

// simple class for timing small segments of code
// and calculating the time delta in loops
class Clock
{
public:
    Clock()
    {
        last_update_time = micros();
    }

    void reset()
    {
        time_seconds = 0;
        time_micros = 0;
        last_update_time = micros();
    }

    void step()
    {
        uint32_t new_time = micros();
        uint32_t elapsed = new_time - last_update_time;
        last_update_time = new_time;
        time_seconds += float(elapsed) * 1e-6f;
        time_micros += elapsed;
        dt_seconds = float(elapsed) * 1e-6f;
        dt_micros = elapsed;
    }

    float time_seconds = 0;
    uint32_t time_micros = 0;
    float dt_seconds = 0;
    uint32_t dt_micros = 0;
    uint32_t last_update_time = 0;
};

#endif // STIM_CLOCK_H
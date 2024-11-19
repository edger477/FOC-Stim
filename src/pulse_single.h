#ifndef STIM_PULSE_SINGLE_H
#define STIM_PULSE_SINGLE_H

#include <stdint.h>
#include <math.h>

#include <SimpleFOC.h>

#include "config.h"

class SingleChannelPulse
{
public:
  SingleChannelPulse() {};

  void create_pulse(float amplitude, float carrier_freq, float pulse_width)
  {
    pulse_duration = 1.f / carrier_freq * pulse_width; // seconds
    for (int i = 0; i < ONEPHASE_PULSE_BUFFER_SIZE; i++)
    {
      float t = (i * pulse_duration) / float(ONEPHASE_PULSE_BUFFER_SIZE);
      float wave_current = cosf(t * carrier_freq * _2PI);
      float wave_current_d = -sinf(t * carrier_freq * _2PI) * carrier_freq * _2PI;
      float pulse_current = -cosf(t * (carrier_freq / pulse_width) * _2PI) * 0.5f + 0.5f;
      float pulse_current_d = sinf(t * (carrier_freq / pulse_width) * _2PI) * 0.5f * (carrier_freq / pulse_width) * _2PI;
      float desired_current = amplitude * wave_current * pulse_current;
      float rate_of_current_change = amplitude * (wave_current * pulse_current_d + wave_current_d * pulse_current);

      x[i] = desired_current;
      dx[i] = rate_of_current_change;
    }
  }

  void get(float t, float *x_out, float *dx_out)
  {
    uint32_t p = (t / pulse_duration) * ONEPHASE_PULSE_BUFFER_SIZE;
    if (p >= ONEPHASE_PULSE_BUFFER_SIZE)
    {
      *x_out = 0;
      *dx_out = 0;
    }
    else
    {
      *x_out = x[p];
      *dx_out = dx[p];
    }
  }

  float pulse_duration;
  float x[ONEPHASE_PULSE_BUFFER_SIZE];
  float dx[ONEPHASE_PULSE_BUFFER_SIZE];
};

#endif // STIM_PULSE_SINGLE_H
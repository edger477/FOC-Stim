#ifndef FOCSTIM_TRACE_H
#define FOCSTIM_TRACE_H

#include <stdint.h>
#include <Arduino.h>

struct MainLoopTraceLine
{
    uint32_t t_start;
    uint32_t dt_compute;
    uint32_t dt_play;
    uint32_t dt_stabilize;
    uint32_t dt_logs;
    uint32_t mrac_iters;
    float v_drive_max;
    float max_recorded_current_neutral;
    float max_recorded_current_left;
    float max_recorded_current_right;
    float amplitude;
    float R_neutral, R_left, R_right, L;
};

class Trace
{
    static const int MAINLOOP_NUM_ENTRIES = 32;

public:
    Trace() {}

    MainLoopTraceLine *next_main_loop_line()
    {
        MainLoopTraceLine *p = &main_loop_trace[main_loop_trace_index];
        main_loop_trace_index = (main_loop_trace_index + 1) % MAINLOOP_NUM_ENTRIES;
        *p = {};
        return p;
    }

    void print_mainloop_trace()
    {
        Serial.printf("mainloop timings:\r\n");
        Serial.printf("     start|   compute|      play| stabilize|      logs|     niter|\r\n");
        for (int i = 0; i < MAINLOOP_NUM_ENTRIES; i++)
        {
            MainLoopTraceLine *p = &main_loop_trace[(i + main_loop_trace_index) % MAINLOOP_NUM_ENTRIES];
            Serial.printf("%10lu %10lu %10lu %10lu %10lu %10lu\r\n",
                          p->t_start,
                          p->dt_compute,
                          p->dt_play,
                          p->dt_stabilize,
                          p->dt_logs,
                          p->mrac_iters);
        }
        Serial.println();
        Serial.printf("mainloop signals:\r\n");
        Serial.printf("   v_drive|   max I_N|   max I_L|   max I_R| amplitude|\r\n");
        for (int i = 0; i < MAINLOOP_NUM_ENTRIES; i++)
        {
            MainLoopTraceLine *p = &main_loop_trace[(i + main_loop_trace_index) % MAINLOOP_NUM_ENTRIES];
            Serial.printf("%10f %10f %10f %10f %10f\r\n",
                          p->v_drive_max,
                          p->max_recorded_current_neutral,
                          p->max_recorded_current_left,
                          p->max_recorded_current_right,
                          p->amplitude);
        }
        Serial.println();
        Serial.printf("mainloop MRAC:\r\n");
        Serial.printf(" R_neutral|    R_left|   R_right|         L|\r\n");
        for (int i = 0; i < MAINLOOP_NUM_ENTRIES; i++)
        {
            MainLoopTraceLine *p = &main_loop_trace[(i + main_loop_trace_index) % MAINLOOP_NUM_ENTRIES];
            Serial.printf("%10f %10f %10f %10f\r\n",
                          p->R_neutral,
                          p->R_left,
                          p->R_right,
                          p->L);
        }
        Serial.println();
    }

    MainLoopTraceLine main_loop_trace[MAINLOOP_NUM_ENTRIES] = {};

    unsigned main_loop_trace_index = 0;
};

#endif // FOCSTIM_TRACE_H
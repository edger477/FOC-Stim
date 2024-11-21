#ifndef FOCSTIM_EMERGENCY_STOP_H
#define FOCSTIM_EMERGENCY_STOP_H

class BLDCDriver6PWM;
class CurrentSense;

class EmergencyStop
{
public:
    EmergencyStop();

    void init(BLDCDriver6PWM *driver, CurrentSense *current_sense, void (*debug_fn)());

    // check if current sensors are within safe limits
    void check_current_limits();
    void check_current_limits(float a, float b, float c);

    // check if VBUS is within safe limits
    void check_vbus();

    // Put hardware in safe state, then call the debug function
    void trigger_emergency_stop();

    float max_recorded_current_a = 0;
    float max_recorded_current_b = 0;
    float max_recorded_current_c = 0;

    BLDCDriver6PWM *driver = nullptr;
    CurrentSense *currentSense = nullptr;
    void (*debug_fn)();
};

#endif // FOCSTIM_EMERGENCY_STOP_H
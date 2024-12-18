#ifndef FOCSTIM_THREEPHASE_DRIVER
#define FOCSTIM_THREEPHASE_DRIVER

#include <SimpleFOC.h>

class BLDCDriver;
class CurrentSense;
class EmergencyStop;


struct ThreePhaseCurrents {
    float neutral;
    float left;
    float right;

    void normalize() {
        float midpoint = (neutral + left + right) / 3;
        neutral -= midpoint;
        left -= midpoint;
        right -= midpoint;
    }
};

/*
B-G431B notes:
The pwm is configured in center aligned mode, so it counts up and down.
Output compare mode is used.
At pwm peak (TIM1->CNT ~2100) all low side mosfets are turned on, all phases connected to ground.
high side mosfets are turned on when timer value is less than the output compare value.

At pwm peak the following occurs:
 - The current sense DMA is triggered
 - The output compare registers are refreshed with the new values written in the TIM1->OCRx registers (preload)
 - The timer interrupt is called
*/
class ThreePhaseDriver {
public:
    ThreePhaseDriver() = default;

    void init(BLDCDriver *driver, CurrentSense *current_sense);

    // race-condition-free way to set all phases to ground, minimum energy consumption.
    void pwm_to_ground();

    // race-condition-fee way to set all phases to center, usually (6, 6, 6).
    void pwm_to_center();

    // set all phases floating
    // TODO: implement
    // void pwm_to_floating();

    // set pwm voltage
    // Do not set the pwm voltage higher than STIM_PWM_MAX_DUTY_CYCLE * STIM_PSU_VOLTAGE
    void set_pwm_voltage(float voltage_neutral, float voltage_left, float voltage_right);

    // pwm-peak interrupt
    void attach_interrupt(std::function<void()> fn);
    void detach_interrupt();

    // Modify the current sense offsets to account for temperature drift
    // in the current sense circuit. Call 1-100 times per second.
    void adjust_offsets();

    ThreePhaseCurrents get_currents();

    BLDCDriver* driver = nullptr;
    CurrentSense* current_sense = nullptr;
};


#endif // FOCSTIM_THREEPHASE_DRIVER
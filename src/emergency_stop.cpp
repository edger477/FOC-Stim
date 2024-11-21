#include "emergency_stop.h"

#include <SimpleFOC.h>
#include <Arduino.h>

#include "utils.h"
#include "config.h"

EmergencyStop::EmergencyStop()
{
}

void EmergencyStop::init(BLDCDriver6PWM *driver, CurrentSense *current_sense, void (*debug_fn)())
{
    this->driver = driver;
    this->currentSense = current_sense;
    this->debug_fn = debug_fn;
}

void EmergencyStop::check_current_limits()
{
    PhaseCurrent_s currents = currentSense->getPhaseCurrents();
    float mid = (currents.a + currents.b + currents.c) / 3;
    float a = currents.a - mid;
    float b = currents.b - mid;
    float c = currents.c - mid;
    check_current_limits(a, b, c);
}

void EmergencyStop::check_current_limits(float a, float b, float c)
{
    max_recorded_current_a = abs(max_recorded_current_a) > abs(a) ? max_recorded_current_a : a;
    max_recorded_current_b = abs(max_recorded_current_b) > abs(b) ? max_recorded_current_b : b;
    max_recorded_current_c = abs(max_recorded_current_c) > abs(c) ? max_recorded_current_c : c;
    if (max({abs(a), abs(b), abs(c)}) > ESTOP_CURRENT_LIMIT)
    {
        trigger_emergency_stop();
        while (1)
        {
            Serial.printf("current limit exceeded: %f %f %f. Restart device to proceed\r\n", a, b, c);
            delay(5000);
        }
    }
}

void EmergencyStop::check_vbus()
{
    float vbus = read_vbus(currentSense);
    if (vbus > STIM_PSU_VOLTAGE_MAX || vbus < STIM_PSU_VOLTAGE_MIN)
    {
        trigger_emergency_stop();
        while (1)
        {
            Serial.printf("V_BUS over/undervoltage detected %f. Current V_BUS=%f. Restart device to proceed\r\n",
                          vbus, read_vbus(currentSense));
            delay(5000);
        }
    }
}

void EmergencyStop::trigger_emergency_stop()
{
    // drain the inductors as fast as possible.
    driver->setPwm(0, 0, 0);
    delayMicroseconds(200);
    // disable all phases just in case one of the mosfets blew up.
    driver->setPhaseState(PHASE_OFF, PHASE_OFF, PHASE_OFF);
    driver->setPwm(0, 0, 0);
    debug_fn();
}

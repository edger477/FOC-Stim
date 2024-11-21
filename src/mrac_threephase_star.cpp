#include "mrac_threephase_star.h"

#include <math.h>

#include <SimpleFOC.h>

#include "emergency_stop.h"
#include "utils.h"

MRACThreephaseStar::MRACThreephaseStar()
{
}

void MRACThreephaseStar::init(BLDCDriver *driver, CurrentSense *currentSense, EmergencyStop *emergencyStop)
{
    this->driver = driver;
    this->currentSense = currentSense;
    this->emergencyStop = emergencyStop;
}

void MRACThreephaseStar::iter(
    float desired_current_neutral, float desired_rate_of_current_change_neutral,
    float desired_current_left, float desired_rate_of_current_change_left)
{
    uint32_t current_time = micros();
    float dt = float(current_time - last_update) * 1e-6f;
    last_update = current_time;

    // PID update
    float pid_error_a = xHat_a - desired_current_neutral;
    float pid_error_b = xHat_b - desired_current_left;
    pid_a_p = lerp(0.5f, pid_a_p, pid_error_a * PID_Kp);
    pid_b_p = lerp(0.5f, pid_b_p, pid_error_b * PID_Kp);
    pid_a_i += pid_error_a * PID_Ki;
    pid_b_i += pid_error_b * PID_Ki;

    // PID control
    desired_rate_of_current_change_neutral -= pid_a_p + pid_a_i;
    desired_rate_of_current_change_left -= pid_b_p + pid_b_i;

    // reference model control signal = r, calculate from xdot = Am * x + Bm * r
    float r_a = R0 * desired_current_neutral + L0 * desired_rate_of_current_change_neutral;
    float r_b = R0 * desired_current_left + L0 * desired_rate_of_current_change_left;

    // real hardware control signal = u = -Kx * xHat + Kr * r
    float u_a = -Kx11 * xHat_a + -Kx12 * xHat_b + Kr * r_a;
    float u_b = -Kx21 * xHat_a + -Kx22 * xHat_b + Kr * r_b;
    // HACK HACK HACK
    if (desired_current_neutral == 0 && desired_current_left == 0)
    {
        u_a = Kr * r_a;
        u_b = Kr * r_b;
    }
    float u_c = -(u_a + u_b);

    float u_high = max(u_a, max(u_b, u_c));
    float u_low = min(u_a, min(u_b, u_c));

    // check voltage limits
    if ((u_high - u_low) > (STIM_PSU_VOLTAGE * STIM_PWM_MAX_DUTY_CYCLE))
    {
        float q = (STIM_PSU_VOLTAGE * STIM_PWM_MAX_DUTY_CYCLE) / (u_high - u_low);
        r_a *= q;
        u_a *= q;
        r_b *= q;
        u_b *= q;
        u_c *= q;
        u_high *= q;
        u_low *= q;
    }
    // log stats
    v_drive_max = max(v_drive_max, u_high - u_low);

    // Control hardware
    // use pwm centered, for maximum waveform smoothness
    // but if duty cycle is high shift the waveform down to help pwm rejection of current sense circuit.
    float center = driver->voltage_power_supply / 2;
    float offset = min(0.f, (STIM_PSU_VOLTAGE * STIM_PWM_MAX_DUTY_CYCLE) - (center + u_high));
    center = center + offset;
    driver->setPwm(
        center + u_c, // right, output closest to the pot
        center + u_a, // neutral, center output
        center + u_b  // left
    );

    // measure and filter the hardware current
    PhaseCurrent_s currents = currentSense->getPhaseCurrents();
    float midpoint = (currents.a + currents.b + currents.c) / 3;
    float x_a = currents.b - midpoint;  // neutral
    float x_b = currents.c - midpoint;  // left
    float x_c = currents.a - midpoint;  // right

    // bookkeeping (optional)
    neutral_abs_sum += abs(x_a);
    left_abs_sum += abs(x_b);
    right_abs_sum += abs(x_c);
    neutral_sum += x_a;
    left_sum += x_b;
    right_sum += x_c;

    // calculate error and perform the update step
    // based on lagged system state.
    float error_a = x_a - state_lag1.xHat_a;
    float error_b = x_b - state_lag1.xHat_b;
    if ((dt < 100e-6f) && ((abs(desired_current_neutral) > .01f) || (abs(desired_current_left) > 0.01f)))
    {
        const float speed = 2.f;
        const float gamma1 = -.1f * speed * P * B * dt;
        const float gamma2 = .1f * speed * P * B * dt;

        // do kx += dt * gamma * x * err^T * P * B;
        Kx11 += gamma1 * (state_lag1.xHat_a * error_a);
        Kx12 += gamma1 * (state_lag1.xHat_a * error_b);
        Kx21 += gamma1 * (state_lag1.xHat_b * error_a);
        Kx22 += gamma1 * (state_lag1.xHat_b * error_b);

        // do kr += dt * gamma * r * err^T * P * B;
        Kr += gamma2 * (state_lag1.r_a * error_a);
        Kr += gamma2 * (state_lag1.r_b * error_b);

        // constrain r3 = r3_alt
        float err = ((Kx22 + 2 * Kx12) - (Kx11 + 2 * Kx21)) / 6;
        Kx11 += 1.0f * err;
        Kx12 -= 2.0f * err;
        Kx21 += 2.0f * err;
        Kx22 -= 1.0f * err;

        Kr = _constrain(Kr, L_min / L0, L_max / L0);
        // TODO: needs fix.
        Kx11 = _constrain(Kx11, R0 * Kr - R_max, R0 * Kr - R_min);                           // constrain R1 between R_min and R_max
        Kx22 = _constrain(Kx22, R0 * Kr - R_max, R0 * Kr - R_min);                           // constrain R2
        Kx12 = _constrain(Kx12, (R0 * Kr - Kx22 - R_max) / 2, (R0 * Kr - Kx22 - R_min) / 2); // constrain R3
        Kx21 = _constrain(Kx21, (R0 * Kr - Kx11 - R_max) / 2, (R0 * Kr - Kx11 - R_min) / 2); // constrain R3
    }

    // system state update, fast convergance to true system state.
    xHat_a += 1.0f * error_a;
    xHat_b += 1.0f * error_b;

    // update the reference model state; x = Ax + By
    xHat_a += min(dt, 50e-6f) * (A * xHat_a + B * r_a);
    xHat_b += min(dt, 50e-6f) * (A * xHat_b + B * r_b);

    // store debug stats
    if (debug_counter >= DEBUG_NUM_ENTRIES)
    {
        debug_counter = 0;
    }
    debug[debug_counter] = {
        r_a, r_b,
        u_a, u_b,
        currents.b, currents.c, currents.a,
        xHat_a, xHat_b,
        desired_current_neutral, desired_current_left,
    };
    debug_counter++;

    // check safety limits
    emergencyStop->check_current_limits(x_a, x_b, x_c);

    // store system state
    state_lag1 = {r_a, r_b, xHat_a, xHat_b};
}

// Configure the hardware in a safe state
// before running code that is slow.
void MRACThreephaseStar::prepare_for_idle()
{
    driver->setPwm(
        driver->voltage_power_supply / 2,
        driver->voltage_power_supply / 2,
        driver->voltage_power_supply / 2);
    xHat_a = 0;
    xHat_b = 0;
}

float MRACThreephaseStar::estimate_inductance()
{
    return L0 * Kr;
}

float MRACThreephaseStar::estimate_r1()
{ // neutral
    return Kr * R0 - Kx11 + Kx21;
}

float MRACThreephaseStar::estimate_r2()
{ // left
    return Kr * R0 - Kx22 + Kx12;
}

float MRACThreephaseStar::estimate_r3()
{ // right
    // these two are equivalent provided the model matches
    return Kr * R0 - Kx11 - 2 * Kx21;
    // return Kr * R0 - Kx22 - 2 * Kx12;
}

float MRACThreephaseStar::estimate_r3_alt()
{
    // these two are equivalent provided the model matches
    // return Kr * R0 - Kx11 - 2 * Kx21;
    return Kr * R0 - Kx22 - 2 * Kx12;
}

void MRACThreephaseStar::print_debug_stats()
{
    {
        Serial.printf("MRAC debug info:\r\n");
        Serial.printf("R0 =  %f\r\n", R0);
        Serial.printf("L0 =  %f\r\n", L0);
        Serial.printf("Kr =  %f\r\n", Kr);
        Serial.printf("Kx = [%f  %f]\r\n", Kx11, Kx12);
        Serial.printf("     [%f  %f]\r\n", Kx21, Kx22);
        Serial.printf("R1 =  %f\r\n", estimate_r1());
        Serial.printf("R2 =  %f\r\n", estimate_r2());
        Serial.printf("R3 =  %f\r\n", estimate_r3());
        Serial.printf("L  =  %f\r\n", estimate_inductance());
        Serial.printf("Pia= %f %f\r\n", pid_a_p, pid_a_i);
        Serial.printf("Pib= %f %f\r\n", pid_b_p, pid_b_i);
        for (unsigned n = 0; n < DEBUG_NUM_ENTRIES; n++)
        {
            int i = (n + debug_counter + DEBUG_NUM_ENTRIES) % DEBUG_NUM_ENTRIES;
            Serial.printf("%u ", n);
            Serial.printf("%f %f ", debug[i].r_a, debug[i].r_b);
            Serial.printf("%f %f ", debug[i].u_a, debug[i].u_b);
            Serial.printf("%f %f %f ", debug[i].x_a, debug[i].x_b, debug[i].x_c);
            Serial.printf("%f %f ", debug[i].xHat_a, debug[i].xHat_b);
            Serial.printf("%f %f ", debug[i].desired_a, debug[i].desired_b);
            // Serial.printf("%f %f ", debug[i].x_a - debug[i].xHat_a, debug[i].x_b - debug[i].xHat_b);
            Serial.println();
        }
        Serial.println();
    }
}
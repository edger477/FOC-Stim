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

void MRACThreephaseStar::pulse_begin()
{
    TIM_TypeDef *tim = TIM1;
    // Temporarily disable the update event to ensure atomic update
    // of all 3 pwm timer outputs, preventing transients.
    SET_BIT(tim->CR1, TIM_CR1_UDIS_Msk);
    float center = driver->voltage_power_supply / 2;
    driver->setPwm(center, center, center);
    CLEAR_BIT(tim->CR1, TIM_CR1_UDIS_Msk);
}

void MRACThreephaseStar::pulse_end()
{
    // wait until lines stabilize
    while (is_stabilized <= 5) {
        iter(0, 0);
    }

    TIM_TypeDef *tim = TIM1;
    // Temporarily disable the update event to ensure atomic update
    // of all 3 pwm timer outputs, preventing transients.
    SET_BIT(tim->CR1, TIM_CR1_UDIS_Msk);
    driver->setPwm(0, 0, 0);    // connect all to ground.
    CLEAR_BIT(tim->CR1, TIM_CR1_UDIS_Msk);

    xHat_a = 0;
    xHat_b = 0;
}

void MRACThreephaseStar::iter(float desired_current_neutral, float desired_current_left)
{
    uint32_t current_time = micros();
    float dt = float(current_time - last_update) * 1e-6f;
    last_update = current_time;

    // calculate r from: desired = xhat + dt * (A * xhat + B * r)
    float r_a = ((desired_current_neutral - xHat_a) / min(dt, 50e-6f) - A * xHat_a) * (1/B);
    float r_b = ((desired_current_left - xHat_b) / min(dt, 50e-6f) - A * xHat_b) * (1/B);

    // real hardware control signal = u = -Kx * xHat + Kr * r
    float u_ad = (+2 * Ka * xHat_a) - (1 * Kb * xHat_b) + (Kc * xHat_a + Kc * xHat_b);
    float u_bd = (-1 * Ka * xHat_a) + (2 * Kb * xHat_b) + (Kc * xHat_a + Kc * xHat_b);
    float u_a = Kr * r_a - u_ad;
    float u_b = Kr * r_b - u_bd;
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
    if ((abs(x_a) + abs(x_b) + abs(x_c)) < 0.1f) {
        is_stabilized++;
    } else {
        is_stabilized = 0;
    }

    // bookkeeping (optional)
    current_squared_a += min(50e-6f, dt) * abs(x_a * x_a);
    current_squared_b += min(50e-6f, dt) * abs(x_b * x_b);
    current_squared_c += min(50e-6f, dt) * abs((x_a + x_b) * (x_a + x_b));


    // calculate error and perform the update step
    // based on lagged system state.
    float error_a = x_a - state_lag1.xHat_a;
    float error_b = x_b - state_lag1.xHat_b;
    if ((dt < 100e-6f) && ((abs(desired_current_neutral) > .01f) || (abs(desired_current_left) > 0.01f)))
    {
        const float speed = 10.f;
        const float gamma1 = -.1f * (speed * P * B * dt);
        const float gamma2 = .1f * (speed * P * B * dt);

        // do Kx += dt * gamma * x * err^T * P * B;
        // code below does not follow the textbook equation, but seems to work best....
        Ka += gamma1 * (state_lag1.xHat_a * error_a);
        Kb += gamma1 * (state_lag1.xHat_b * error_b);
        Kc += gamma1 * ((state_lag1.xHat_a + state_lag1.xHat_b) * (error_a + error_b)); // = xHat_c * error_c

        // do kr += dt * gamma * r * err^T * P * B;
        Kr += gamma2 * (state_lag1.r_a * error_a);
        Kr += gamma2 * (state_lag1.r_b * error_b);
        Kr += gamma2 * (state_lag1.r_a + state_lag1.r_b) * (error_a + error_b); // = r_c * error_c

        Kr = _constrain(Kr, L_min / L0, L_max / L0);
        const float minimum = (R0 * Kr - R_max) / 3;
        const float maximum = (R0 * Kr - R_min) / 3;
        Ka = _constrain(Ka, minimum, maximum);
        Kb = _constrain(Kb, minimum, maximum);
        Kc = _constrain(Kc, minimum, maximum);
    }

    // mixin measurements, effectively luenberger observer.
    static constexpr float luenberger_gain = 1.f - expf(-_2PI * observer_cutoff_frequency / estimated_loop_frequency); // 1 - L = exp(-w * T)
    xHat_a += luenberger_gain * error_a;
    xHat_b += luenberger_gain * error_b;

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

float MRACThreephaseStar::estimate_inductance()
{
    return L0 * Kr;
    // The true inductance is actually dt / ln(1 - dt / (L0 * Kr))
}

float MRACThreephaseStar::estimate_resistance_neutral()
{
    return (R0 * Kr - 3 * Ka);
}

float MRACThreephaseStar::estimate_resistance_left()
{
    return (R0 * Kr - 3 * Kb);
}

float MRACThreephaseStar::estimate_resistance_right()
{
    return (R0 * Kr - 3 * Kc);
}

void MRACThreephaseStar::print_debug_stats()
{
    {
        Serial.printf("MRAC debug info:\r\n");
        Serial.printf("R0 =  %f\r\n", R0);
        Serial.printf("L0 =  %f\r\n", L0);
        Serial.printf("Kr =  %f\r\n", Kr);
        Serial.printf("Ka =  %f\r\n", Ka);
        Serial.printf("Kb =  %f\r\n", Kb);
        Serial.printf("Kc =  %f\r\n", Kc);
        Serial.printf("Ra =  %f\r\n", estimate_resistance_neutral());
        Serial.printf("Rb =  %f\r\n", estimate_resistance_left());
        Serial.printf("Rc =  %f\r\n", estimate_resistance_right());
        Serial.printf("L  =  %f\r\n", estimate_inductance());
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
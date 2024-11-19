#ifndef FOCSTIM_MRAC_1D
#define FOCSTIM_MRAC_1D

#include <stdint.h>
#include <math.h>

#include <SimpleFOC.h>

// ###################################
// MRAC model reference adaptive control.
// the equation is    v = R*I + L * di/dt
// rearrange to   di/dt = -R/L * I + 1/L * v
// or                dx = A*x + B*u
// with A = -R/L and B = 1/L
//
// Model update:
// xM = dt * (A * xM + B * r)     r = control input, xM = model state
//
// Control law (default case):
// u = -K1 * x + K2 * r           u = system control variable, x = measured value
// e = x - xM                     error
//
// luenberger observer:
// xHat = dt * (A * xHat + B * r + L * (x - xHat))      L = gain > 0
// Now replace x with xHat in control law and error.
//
// luenberger observer with delayed measurements:
// xHat = dt * (A * xHat + B * r + L * (x - xHat[n-1]))
//
// basic resources on MRAC:
// https://mathworks.com/help/slcontrol/ug/model-reference-adaptive-control.html
// https://www.youtube.com/watch?v=GBBXZXmb8UE
//
// details on the luenberger observer:
// https://iopscience.iop.org/article/10.1088/1742-6596/1418/1/012017/pdf
// https://www.youtube.com/watch?v=EWeFxseU6g4
//
// Good overview of different MRAC variations:
// https://www.researchgate.net/publication/330199008_Comparison_of_MRAC_and_L1_Adaptive_Controllers_for_a_Gimbaled_Mini-Free_Flyer

class MRAC_1D
{
public:
    const float L0 = 500e-6f;
    const float L_min = 100e-6f;
    const float L_max = 5000e-6f;
    const float R0 = 4.f;
    const float R_min = 1.f;
    const float R_max = 15.f;

    const float A = -R0 / L0;
    const float B = 1 / L0;
    const float P = -1;

    MRAC_1D() {

    };

    void init(BLDCDriver *driver, CurrentSense *currentSense)
    {
        this->driver = driver;
        this->currentSense = currentSense;
    }

    void iter(float dt, float desired_current, float desired_rate_of_current_change)
    {
        loop_counter++;

        // reference model control signal
        float r = R0 * desired_current + L0 * desired_rate_of_current_change;

        // real hardware control signal
        float u = -Kx * xHat + Kr * r;

        // bookkeeping, optional
        if (abs(u) > abs(max_volts))
        {
            max_volts = u;
        }

        // check voltage limit
        if (abs(u) > 8)
        {
            float f = 8 / abs(u);
            r *= f;
            u *= f;
        }

        // update the reference model state;
        xHat += dt * (A * xHat + B * r);

        // store state for later
        history[(loop_counter % 16)] = {r, xHat};

        // Control hardware
        float center = driver->voltage_limit / 2;
        u = _constrain(u, -8, 8);
        driver->setPwm(
            center + u / 2,
            center - u / 2,
            center);

        // measure the hardware current
        PhaseCurrent_s phase_current_meas = currentSense->getPhaseCurrents();
        float p1_err = phase_current_meas.a - p1_avg; // high pass filter to account for temperature drift in the sense circuit
        float p2_err = phase_current_meas.b - p2_avg;
        p1_avg += (p1_err) * (1 / 25000.f);
        p2_avg += (p2_err) * (1 / 25000.f);
        float x = (p1_err - p2_err) / 2;

        const uint32_t lags = 1;
        state_history_t prev_state = history[(loop_counter - lags) % 16];

        float error = x - prev_state.predicted_state;
        xHat += 0.1f * error;

        // update parameters (gradient descent)
        if (dt < 100e-6f)
        {
            const float gamma1 = -.1f / 2;
            const float gamma2 = .1f / 4 / 2;
            Kx += dt * gamma1 * (prev_state.predicted_state * error * P * B);
            Kr += dt * gamma2 * (prev_state.r * error * P * B);

            Kr = _constrain(Kr, L_min / L0, L_max / L0);
            Kx = _constrain(Kx, R0 * Kr - R_max, R0 * Kr - R_min);
        }
    }

    float estimate_inductance()
    {
        return L0 * Kr;
    }

    float estimate_resistance()
    {
        return R0 * Kr - Kx;
    }

    BLDCDriver *driver;
    CurrentSense *currentSense;
    unsigned loop_counter;

    float Kr = 1;   // feedforward gain, true inductance = L = L0 * Kr
    float Kx = 0;   // feedback gain, true resistance = R = R0 * Kr - Kx;
    float xHat = 0; // estimated system current

    float p1_avg = 0; // used for high-pass filtering current measurements
    float p2_avg = 0;

    float max_volts = 0;

    struct state_history_t
    {
        float r; // control signal
        float predicted_state;
    } history[16] = {};
};

#endif // FOCSTIM_MRAC_1D
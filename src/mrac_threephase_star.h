#ifndef FOCSTIM_MRAC_THREEPHASE_STAR_H
#define FOCSTIM_MRAC_THREEPHASE_STAR_H

#include <stdint.h>

#include "config.h"

class BLDCDriver;
class CurrentSense;
class EmergencyStop;

/*
MRAC model for threephase system with:
star connection with floating neutral
each leg has the same inductance
eacn leg has it's own resistance


Va, Vb, Vc      = driving voltage.
N               = neutral point voltage (unknown)
Ra, Rb, Rc      = leg resistance
L               = leg inductance
Ia, Ib, Ic      = leg current
Iad, Ibd, Icd   = derivative of the leg current


Modelling:
Va - N = Ra * Ia + L * Iad       and vice-versa for b, c
Consider:
(Va - N) - (Vb - N) = Ra * Ia + L * Iad - Rb * Ib - L * Ibd
(Vb - N) - (Vc - N) = Rb * Ib + L * Ibd - Rc * Ic - L * Icd
Substitude C = -(A + B) and Ic = -(Ia + Ib) and Icd = -(Iad + Ibd) then solve:

    Va, Vb, N = symbols('Va, Vb, N')
    Ra, Rb, Rc = symbols('Ra, Rb, Rc')
    Ia, Ib = symbols('Ia, Ib')
    Iad, Ibd = symbols('Iad, Ibd')
    L = symbols('L')
    Vc = -(Va + Vb)
    Ic = -(Ia + Ib)
    Icd = -(Iad + Ibd)
    p1 = Eq(Va - Vb, Ra * Ia + L * Iad - Rb * Ib - L * Ibd)
    p2 = Eq(Vb - Vc, Rb * Ib + L * Ibd - Rc * Ic - L * Icd)
    solution = solve([p1, p2], [Iad, Ibd])
    pprint(solution[Iad])
    pprint(solution[Ibd])

          /------------ matrix A ----------\     /- matrix B -\    /-- input voltage --\
[[Iad]  = 1/(3*L) * [[-2Ra - Rc,    Rb - Rc]   + 1/L [[1,    0]  * [[A]
 [Ibd]]              [  Ra - Rc,  -2Rb - Rc]]         [0,    1]]    [B]]

Model matching conditions:
Bm = B * Kr           Kr a scalar
Am = A - B * Kx       Kx a 2x2 matrix

Control law:
u = -Kx * x + Kr * r
*/
class MRACThreephaseStar
{
public:
    static constexpr float L0 = MODEL_INDUCTANCE_INIT;
    static constexpr float L_min = MODEL_INDUCTANCE_MIN;
    static constexpr float L_max = MODEL_INDUCTANCE_MAX;
    static constexpr float R0 = MODEL_RESISTANCE_INIT;
    static constexpr float R_min = MODEL_RESISTANCE_MIN;
    static constexpr float R_max = MODEL_RESISTANCE_MAX;

    static constexpr float PID_Kp = 1.0f / L0;
    static constexpr float PID_Ki = .1f / L0;

    static constexpr float A = -R0 / L0;
    static constexpr float B = 1 / L0;
    static constexpr float P = -1;

    static constexpr int DEBUG_NUM_ENTRIES = 450; // very RAM hungry

    MRACThreephaseStar();

    void init(BLDCDriver *driver, CurrentSense *currentSense, EmergencyStop *emergencyStop);

    // call this as fast as possible
    void iter(
        float desired_current_neutral, float desired_rate_of_current_change_neutral,
        float desired_current_left, float desired_rate_of_current_change_left);

    // call this if you plan to stop calling iter() as fast as possible
    // to temporarily configure the hardware in a safe state.
    void prepare_for_idle();

    float estimate_inductance();
    float estimate_r1();     // neutral
    float estimate_r2();     // left
    float estimate_r3();     // right
    float estimate_r3_alt(); // right

    void print_debug_stats();

    unsigned debug_counter = 0;
    uint32_t last_update = 0;

    BLDCDriver *driver = nullptr;
    CurrentSense *currentSense = nullptr;
    EmergencyStop *emergencyStop = nullptr;

    // PI loop variables
    float pid_a_p = 0;
    float pid_b_p = 0;
    float pid_a_i = 0;
    float pid_b_i = 0;

    // MRAC variables.
    float Kr = 1;                 // feedforward gain, true inductance = L = L0 * Kr
    float Kx11 = 0;               // = 1/3 * (3*Kr * R0 - 2*Ra - Rc)
    float Kx12 = 0;               // = 1/3 * (Rb - Rc)
    float Kx21 = 0;               // = 1/3 * (Ra - Rc)
    float Kx22 = 0;               // = 1/3 * (3*Kr * R0 - 2*Rb - Rc)
    float xHat_a = 0, xHat_b = 0; // estimated system current

    struct state_history_t
    {
        float r_a;
        float r_b;
        float predicted_state_a;
        float predicted_state_b;
    } prev_state = {};

    // log stats
    float neutral_abs_sum = 0;
    float left_abs_sum = 0;
    float right_abs_sum = 0;
    float neutral_sum = 0;
    float left_sum = 0;
    float right_sum = 0;
    float v_drive_max = 0;

    // more log stats
    struct
    {
        float meas_a;
        float meas_b;
        float xhat_a;
        float xhat_b;
        float desired_a;
        float desired_b;
        float va;
        float vb;
        float dt;
    } debug[DEBUG_NUM_ENTRIES] = {};
};

#endif // FOCSTIM_MRAC_THREEPHASE_STAR_H
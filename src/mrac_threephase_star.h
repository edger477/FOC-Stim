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

          /------------ matrix A ----------\             /- matrix B -\
[[Iad]  = 1/(3*L) * [[-2Ra - Rc,    Rb - Rc]  * [[Ia]  + 1/L [[1,    0]  * [[Va]
 [Ibd]]              [  Ra - Rc,  -2Rb - Rc]]    [Ib]]        [0,    1]]    [Vb]]

Reference model:
xd = Am * x + Bm * r

Control law:
u = Kr * r - Kx * x

plant model:
xd = A * x + B * u
xd = A * x + B * [Kr * r - Kx * x]

Model matching conditions:
Bm = B * Kr                 Kr a scalar
Am = A - B * Kx             Kx a 2x2 matrix

After solving model matching condition 1 we find:
L = L0 * Kr

We choose Kx = Ka * [2  0] + Kb * [0 -1] + Kc * [1  1]
                    [-1 0]        [0  2]        [1  1]
After solving model matching condition 2 we find:
R0 * Kr - 3 * Ka = Ra
R0 * Kr - 3 * Kb = Rb
R0 * Kr - 3 * Kc = Rc
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

    static constexpr float estimated_loop_frequency = 28000;
    static constexpr float observer_cutoff_frequency = 300;

    static constexpr float A = -R0 / L0;
    static constexpr float B = 1 / L0;
    static constexpr float P = -1;

    static constexpr int DEBUG_NUM_ENTRIES = 400; // very RAM hungry

    MRACThreephaseStar();

    void init(BLDCDriver *driver, CurrentSense *currentSense, EmergencyStop *emergencyStop);

    // call this as fast as possible
    void iter(float desired_current_neutral, float desired_current_left);

    // call this if you plan to stop calling iter() as fast as possible
    // to temporarily configure the hardware in a safe state.
    void prepare_for_idle();

    float estimate_inductance();
    float estimate_resistance_neutral();
    float estimate_resistance_left();
    float estimate_resistance_right();

    void print_debug_stats();

    unsigned debug_counter = 0;
    uint32_t last_update = 0;

    BLDCDriver *driver = nullptr;
    CurrentSense *currentSense = nullptr;
    EmergencyStop *emergencyStop = nullptr;

    // MRAC variables.
    float Kr = 1;                 // feedforward gain, true inductance = L = L0 * Kr
    float Ka = 0;                 // R0 * Kr - 3 * Ka = Ra
    float Kb = 0;                 // R0 * Kr - 3 * Kb = Rb
    float Kc = 0;                 // R0 * Kr - 3 * Kc = Rc
    float xHat_a = 0, xHat_b = 0; // estimated system current

    float Ka_d = 0;
    float Kb_d = 0;
    float Kc_d = 0;

    struct state_history_t
    {
        float r_a;
        float r_b;
        float xHat_a;
        float xHat_b;
    } state_lag1 = {};

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
        float r_a, r_b;
        float u_a, u_b;
        float x_a, x_b, x_c;
        float xHat_a, xHat_b;
        float desired_a, desired_b;
    } debug[DEBUG_NUM_ENTRIES] = {};
};

#endif // FOCSTIM_MRAC_THREEPHASE_STAR_H
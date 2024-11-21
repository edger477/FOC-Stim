#ifndef FOCSTIM_CONFIG_H
#define FOCSTIM_CONFIG_H

// current limits
#define TCODE_MAX_CURRENT 0.8f                          // in amps
#define ESTOP_CURRENT_LIMIT (TCODE_MAX_CURRENT + 0.20f) // Needs about .20f to account for measurement noise.

// supply voltage and pwm
#define STIM_PWM_FREQ 40000 // switching frequency is twice this frequency
#define STIM_PSU_VOLTAGE 12.0f
#define STIM_PSU_VOLTAGE_MIN 11.5f // e-stop if exceeded
#define STIM_PSU_VOLTAGE_MAX 13.5f // e-stop if exceeded
#define STIM_PWM_MINIMUM_OFF_TIME 4e-6f // limited by pwm-rejection of current sense. Experimentally determined to be 3.15us.
#define STIM_PWM_MAX_DUTY_CYCLE float(1 - STIM_PWM_MINIMUM_OFF_TIME * STIM_PWM_FREQ)

// initial conditions and limits for the model
#define MODEL_RESISTANCE_INIT 2.0f
#define MODEL_RESISTANCE_MIN 1.0f
#define MODEL_RESISTANCE_MAX 15.0f
#define MODEL_INDUCTANCE_INIT 450e-6f
#define MODEL_INDUCTANCE_MIN 100e-6f
#define MODEL_INDUCTANCE_MAX 1500e-6f

// size of precomputed pulse buffer
#define THREEPHASE_PULSE_BUFFER_SIZE 256
#define ONEPHASE_PULSE_BUFFER_SIZE 256

// enable for nicer looking waveforms on the scope. Not safe for humans!
// #define THREEPHASE_PULSE_DEFEAT_RANDOMIZATION

#endif // FOCSTIM_CONFIG_H
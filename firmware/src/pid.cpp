#include "pid.h"
#include <math.h>

// ------------------------------------------------------------
// Initialise PID controller with gains
// ------------------------------------------------------------
void pid_init(PidController *pid, PidGains gains)
{
    pid->gains          = gains;
    pid->integral       = 0.0f;
    pid->prev_error     = 0.0f;
    pid->output         = 0.0f;
}

// ------------------------------------------------------------
// Compute PID output
//   setpoint     — desired velocity (rad/s)
//   measurement  — measured velocity (rad/s)
//   dt           — time step (seconds)
// Returns controller output (PWM-like value, ±output_limit)
// ------------------------------------------------------------
float pid_compute(PidController *pid, float setpoint, float measurement, float dt)
{
    float error = setpoint - measurement;

    // Proportional
    float p_term = pid->gains.kp * error;

    // Integral (with anti-windup)
    pid->integral += error * dt;
    pid->integral  = fmaxf(fminf(pid->integral,  pid->gains.integral_windup),
                          -pid->gains.integral_windup);
    float i_term = pid->gains.ki * pid->integral;

    // Derivative (on measurement to avoid derivative kick)
    float d_term = 0.0f;
    if (dt > 0.0f) {
        float derivative = (measurement - pid->prev_measurement) / dt;
        d_term = pid->gains.kd * derivative;
    }

    // Sum and clamp
    pid->output = p_term + i_term - d_term;
    pid->output = fmaxf(fminf(pid->output,  pid->gains.output_limit),
                       -pid->gains.output_limit);

    // Store state for next cycle
    pid->prev_error       = error;
    pid->prev_measurement = measurement;

    return pid->output;
}

// ------------------------------------------------------------
// Reset integrator and state
// ------------------------------------------------------------
void pid_reset(PidController *pid)
{
    pid->integral          = 0.0f;
    pid->prev_error        = 0.0f;
    pid->prev_measurement  = 0.0f;
    pid->output            = 0.0f;
}

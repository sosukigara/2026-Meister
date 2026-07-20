#ifndef PID_H
#define PID_H

#include "conf_hardware.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    PidGains gains;
    float    integral;
    float    prev_error;
    float    prev_measurement;
    float    output;
} PidController;

void  pid_init(PidController *pid, PidGains gains);
float pid_compute(PidController *pid, float setpoint, float measurement, float dt);
void  pid_reset(PidController *pid);

#ifdef __cplusplus
}
#endif

#endif // PID_H

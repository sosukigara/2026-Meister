#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

void initMotors(void);
void setMotorSpeeds(float left, float right);
void stopMotors(void);

#ifdef __cplusplus
}
#endif

#endif // MOTOR_CONTROL_H

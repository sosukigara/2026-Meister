#include <Arduino.h>
#include "motor_control.h"
#include "conf_hardware.h"

// ------------------------------------------------------------
// Initialise motor PWM channels (LEDC)
// ------------------------------------------------------------
void initMotors(void)
{
    // Left motor
    ledcSetup(MOTOR_PWM_CHAN_LEFT, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION);
    ledcAttachPin(PIN_MOTOR_LEFT_ENA, MOTOR_PWM_CHAN_LEFT);

    pinMode(PIN_MOTOR_LEFT_IN1, OUTPUT);
    pinMode(PIN_MOTOR_LEFT_IN2, OUTPUT);

    digitalWrite(PIN_MOTOR_LEFT_IN1, LOW);
    digitalWrite(PIN_MOTOR_LEFT_IN2, LOW);

    // Right motor
    ledcSetup(MOTOR_PWM_CHAN_RIGHT, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION);
    ledcAttachPin(PIN_MOTOR_RIGHT_ENB, MOTOR_PWM_CHAN_RIGHT);

    pinMode(PIN_MOTOR_RIGHT_IN3, OUTPUT);
    pinMode(PIN_MOTOR_RIGHT_IN4, OUTPUT);

    digitalWrite(PIN_MOTOR_RIGHT_IN3, LOW);
    digitalWrite(PIN_MOTOR_RIGHT_IN4, LOW);
}

// ------------------------------------------------------------
// Set motor speeds
//   speed:  -255 … +255  (negative = reverse)
// ------------------------------------------------------------
void setMotorSpeeds(float left, float right)
{
    // --- LEFT ---
    if (left >= 0.0f) {
        digitalWrite(PIN_MOTOR_LEFT_IN1, HIGH);
        digitalWrite(PIN_MOTOR_LEFT_IN2, LOW);
    } else {
        digitalWrite(PIN_MOTOR_LEFT_IN1, LOW);
        digitalWrite(PIN_MOTOR_LEFT_IN2, HIGH);
    }
    ledcWrite(MOTOR_PWM_CHAN_LEFT, (uint32_t)constrain(fabsf(left), 0.0f, 255.0f));

    // --- RIGHT ---
    if (right >= 0.0f) {
        digitalWrite(PIN_MOTOR_RIGHT_IN3, HIGH);
        digitalWrite(PIN_MOTOR_RIGHT_IN4, LOW);
    } else {
        digitalWrite(PIN_MOTOR_RIGHT_IN3, LOW);
        digitalWrite(PIN_MOTOR_RIGHT_IN4, HIGH);
    }
    ledcWrite(MOTOR_PWM_CHAN_RIGHT, (uint32_t)constrain(fabsf(right), 0.0f, 255.0f));
}

// ------------------------------------------------------------
// Emergency stop
// ------------------------------------------------------------
void stopMotors(void)
{
    ledcWrite(MOTOR_PWM_CHAN_LEFT,  0);
    ledcWrite(MOTOR_PWM_CHAN_RIGHT, 0);

    digitalWrite(PIN_MOTOR_LEFT_IN1,  LOW);
    digitalWrite(PIN_MOTOR_LEFT_IN2,  LOW);
    digitalWrite(PIN_MOTOR_RIGHT_IN3, LOW);
    digitalWrite(PIN_MOTOR_RIGHT_IN4, LOW);
}

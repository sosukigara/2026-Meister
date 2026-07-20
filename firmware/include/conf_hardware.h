#ifndef CONF_HARDWARE_H
#define CONF_HARDWARE_H

#include <stdint.h>

// ============================================================
// Motor Driver PWM Pins (L298N or similar H-bridge)
// ============================================================
#define PIN_MOTOR_LEFT_ENA  32   // PWM — left motor speed
#define PIN_MOTOR_LEFT_IN1  33   // Direction
#define PIN_MOTOR_LEFT_IN2  25   // Direction

#define PIN_MOTOR_RIGHT_ENB 26   // PWM — right motor speed
#define PIN_MOTOR_RIGHT_IN3 27   // Direction
#define PIN_MOTOR_RIGHT_IN4 14   // Direction

// ============================================================
// PWM Configuration (LEDC)
// ============================================================
#define MOTOR_PWM_FREQ      5000   // Hz
#define MOTOR_PWM_RESOLUTION 8     // bits (0–255)
#define MOTOR_PWM_CHAN_LEFT  0     // LEDC channel
#define MOTOR_PWM_CHAN_RIGHT 1

// ============================================================
// Encoder Pins (ESP32 PCNT hardware)
// ============================================================
#define PIN_ENC_LEFT_A       34
#define PIN_ENC_LEFT_B       35
#define PIN_ENC_RIGHT_A      36
#define PIN_ENC_RIGHT_B      39

#define PCNT_UNIT_LEFT       0
#define PCNT_UNIT_RIGHT      1

// ============================================================
// Wheel & Robot Parameters
// ============================================================
#define WHEEL_RADIUS          0.035f   // metres
#define TRACK_WIDTH           0.200f   // metres (wheel separation)
#define ENCODER_TICKS_PER_REV 1200     // 600 PPR * 4 (quadrature)
#define MAX_RPM               150      // motor max speed

// ============================================================
// PID Constants per Motor
// ============================================================
typedef struct {
    float kp;
    float ki;
    float kd;
    float integral_windup;   // max integral accumulation
    float output_limit;      // max output magnitude
} PidGains;

#define PID_GAINS_DEFAULT                        \
    {                                             \
        .kp             = 1.5f,                   \
        .ki             = 0.5f,                   \
        .kd             = 0.1f,                   \
        .integral_windup = 10.0f,                 \
        .output_limit   = 255.0f,                 \
    }

// ============================================================
// IMU I2C Pins (BNO085 — Phase 1.4)
// ============================================================
#define PIN_IMU_SDA         21
#define PIN_IMU_SCL         22

// ============================================================
// Control Loop
// ============================================================
#define CONTROL_LOOP_HZ      20
#define CONTROL_LOOP_MS      50

#endif // CONF_HARDWARE_H

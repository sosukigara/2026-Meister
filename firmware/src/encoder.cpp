#include <Arduino.h>
#include <driver/pcnt.h>
#include "encoder.h"
#include "conf_hardware.h"

// ------------------------------------------------------------
// PCNT configuration structures
// ------------------------------------------------------------
static pcnt_config_t pcnt_left  = { 0 };
static pcnt_config_t pcnt_right = { 0 };

// ------------------------------------------------------------
// Initialise PCNT units for quadrature decoding
// ------------------------------------------------------------
void initEncoders(void)
{
    // ---- Left encoder ----
    pcnt_left.pulse_gpio_num   = PIN_ENC_LEFT_A;
    pcnt_left.ctrl_gpio_num    = PIN_ENC_LEFT_B;
    pcnt_left.channel          = PCNT_CHANNEL_0;
    pcnt_left.unit             = PCNT_UNIT_LEFT;
    pcnt_left.pos_mode         = PCNT_COUNT_DIS;   // will be set dynamically
    pcnt_left.neg_mode         = PCNT_COUNT_DIS;
    pcnt_left.lctrl_mode       = PCNT_MODE_KEEP;
    pcnt_left.hctrl_mode       = PCNT_MODE_REVERSE;
    pcnt_left.counter_h_lim    = INT16_MAX;
    pcnt_left.counter_l_lim    = INT16_MIN;
    pcnt_unit_config(&pcnt_left);

    // Filter glitches (< 10 µs filtered)
    pcnt_set_filter_value(PCNT_UNIT_LEFT, 100);
    pcnt_filter_enable(PCNT_UNIT_LEFT);

    // ---- Right encoder ----
    pcnt_right.pulse_gpio_num  = PIN_ENC_RIGHT_A;
    pcnt_right.ctrl_gpio_num   = PIN_ENC_RIGHT_B;
    pcnt_right.channel         = PCNT_CHANNEL_0;
    pcnt_right.unit            = PCNT_UNIT_RIGHT;
    pcnt_right.pos_mode        = PCNT_COUNT_DIS;
    pcnt_right.neg_mode        = PCNT_COUNT_DIS;
    pcnt_right.lctrl_mode      = PCNT_MODE_KEEP;
    pcnt_right.hctrl_mode      = PCNT_MODE_REVERSE;
    pcnt_right.counter_h_lim   = INT16_MAX;
    pcnt_right.counter_l_lim   = INT16_MIN;
    pcnt_unit_config(&pcnt_right);

    pcnt_set_filter_value(PCNT_UNIT_RIGHT, 100);
    pcnt_filter_enable(PCNT_UNIT_RIGHT);

    // Clear initial counts
    pcnt_counter_clear(PCNT_UNIT_LEFT);
    pcnt_counter_clear(PCNT_UNIT_RIGHT);
}

// ------------------------------------------------------------
// Read raw encoder ticks  (clear counter after read)
// ------------------------------------------------------------
void readEncoders(int32_t *left_ticks, int32_t *right_ticks)
{
    int16_t raw_left  = 0;
    int16_t raw_right = 0;

    pcnt_get_counter_value(PCNT_UNIT_LEFT,  &raw_left);
    pcnt_get_counter_value(PCNT_UNIT_RIGHT, &raw_right);

    *left_ticks  = (int32_t)raw_left;
    *right_ticks = (int32_t)raw_right;

    // Clear counters after read
    pcnt_counter_clear(PCNT_UNIT_LEFT);
    pcnt_counter_clear(PCNT_UNIT_RIGHT);
}

// ------------------------------------------------------------
// Convert raw ticks ± angular speed (rad/s)
// ------------------------------------------------------------
void getWheelSpeeds(int32_t left_ticks, int32_t right_ticks,
                    float *left_speed,  float *right_speed)
{
    // rad/s = (ticks / ticks_per_rev) * 2π * control_loop_hz
    float factor = (2.0f * PI * CONTROL_LOOP_HZ) / (float)ENCODER_TICKS_PER_REV;

    *left_speed  = (float)left_ticks  * factor;
    *right_speed = (float)right_ticks * factor;
}

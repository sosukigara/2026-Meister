#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void initEncoders(void);
void readEncoders(int32_t *left_ticks, int32_t *right_ticks);
void getWheelSpeeds(int32_t left_ticks,  int32_t right_ticks,
                    float *left_speed,   float *right_speed);

#ifdef __cplusplus
}
#endif

#endif // ENCODER_H

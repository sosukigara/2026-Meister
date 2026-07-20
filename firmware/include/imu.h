#ifndef IMU_H
#define IMU_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void initIMU(void);
bool readIMU(float *quat_w, float *quat_x, float *quat_y, float *quat_z);

#ifdef __cplusplus
}
#endif

#endif // IMU_H

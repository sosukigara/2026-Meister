#include <Arduino.h>
#include <Adafruit_BNO08x.h>
#include "imu.h"
#include "conf_hardware.h"

// ------------------------------------------------------------
// Global state
// ------------------------------------------------------------
static Adafruit_BNO08x bno08x;
static bool            imu_ok = false;

// ------------------------------------------------------------
// Initialise BNO085 on I2C bus + enable rotation vector @ 100 Hz
// ------------------------------------------------------------
void initIMU(void)
{
    // Initialise I2C with hardware-configured pins
    Wire.begin(PIN_IMU_SDA, PIN_IMU_SCL);

    if (!bno08x.begin_I2C(BNO08X_DEFAULT_ADDRESS)) {
        Serial.println("BNO085 not found on I2C bus — IMU disabled");
        imu_ok = false;
        return;
    }

    if (!bno08x.enableReport(SH2_ROTATION_VECTOR, 10000)) {   // 10000 µs = 100 Hz
        Serial.println("BNO085: failed to enable rotation-vector report");
        imu_ok = false;
        return;
    }

    imu_ok = true;
    Serial.println("BNO085 ready — rotation vector @ 100 Hz");
}

// ------------------------------------------------------------
// Read latest quaternion  (returns false if no data available)
// ------------------------------------------------------------
bool readIMU(float *quat_w, float *quat_x, float *quat_y, float *quat_z)
{
    if (!imu_ok) return false;

    sh2_SensorValue_t val;
    if (bno08x.getSensorEvent(&val) && val.sensorId == SH2_ROTATION_VECTOR) {
        *quat_w = val.un.rotationVector.real;
        *quat_x = val.un.rotationVector.i;
        *quat_y = val.un.rotationVector.j;
        *quat_z = val.un.rotationVector.k;
        return true;
    }

    return false;
}

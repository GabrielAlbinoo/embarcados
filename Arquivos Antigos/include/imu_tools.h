#ifndef IMU_TOOLS_H
#define IMU_TOOLS_H

#include <stdio.h>
#include "esp_err.h"

typedef struct {
    float acel_x;
    float acel_y;
    float acel_z;
    float giro_x;
    float giro_y;
    float giro_z;
} IMUData;

typedef struct {
    float w;
    float x;
    float y;
    float z;
} Quaternion;

typedef struct {
    float roll;
    float pitch;
    float yaw;
} EulerAngle;

esp_err_t imu_read_data(IMUData *data);
esp_err_t imu_calculate_quaternion(const IMUData *data, Quaternion *quaternion);
esp_err_t imu_calculate_euler_angles(const Quaternion *quaternion, EulerAngle *euler);

#endif

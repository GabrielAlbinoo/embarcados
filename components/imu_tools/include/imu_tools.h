#ifndef IMU_TOOLS_H
#define IMU_TOOLS_H

#include <stdio.h>
#include "esp_err.h"

extern uint8_t imu_dev_addr;

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

esp_err_t imu_read_data(uint8_t reg_addr, uint8_t *data, size_t length);
esp_err_t imu_calculate_quaternion(const IMUData *data, Quaternion *quaternion);
esp_err_t imu_calculate_euler_angles(const Quaternion *quaternion, EulerAngle *euler);

#endif

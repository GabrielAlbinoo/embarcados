#include "imu_tools.h"
#include <math.h>
#include "driver/i2c.h"

#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_GYRO_XOUT_H  0x43
#define MPU6050_PWR_MGMT_1   0x6B

extern uint8_t imu_dev_addr;

esp_err_t imu_read_data(IMUData *data) {
    if (data == NULL) {
        return ESP_FAIL;
    }

    uint8_t accel_data[6];
    uint8_t gyro_data[6];

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (imu_dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, MPU6050_ACCEL_XOUT_H, true);
    i2c_master_start(cmd);  
    i2c_master_write_byte(cmd, (imu_dev_addr << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, accel_data, 6, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);

    if (err != ESP_OK) {
        return err;
    }
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (imu_dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, MPU6050_GYRO_XOUT_H, true);
    i2c_master_start(cmd);  
    i2c_master_write_byte(cmd, (imu_dev_addr << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, gyro_data, 6, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);

    if (err != ESP_OK) {
        return err;
    }

    data->acel_x = (int16_t)((accel_data[0] << 8) | accel_data[1]);
    data->acel_y = (int16_t)((accel_data[2] << 8) | accel_data[3]);
    data->acel_z = (int16_t)((accel_data[4] << 8) | accel_data[5]);

    data->giro_x = (int16_t)((gyro_data[0] << 8) | gyro_data[1]);
    data->giro_y = (int16_t)((gyro_data[2] << 8) | gyro_data[3]);
    data->giro_z = (int16_t)((gyro_data[4] << 8) | gyro_data[5]);

    return ESP_OK;
}

esp_err_t imu_calculate_quaternion(const IMUData *data, Quaternion *quaternion) {
    if (data == NULL || quaternion == NULL) {
        return ESP_FAIL;
    }

    float ax = data->acel_x;
    float ay = data->acel_y;
    float az = data->acel_z;

    float gx = data->giro_x;
    float gy = data->giro_y;
    float gz = data->giro_z;

    float dt = 0.01f;

    float pitch = atan2(-ax, sqrt(ay * ay + az * az));
    float roll = atan2(ay, az);
    float yaw = 0;

    pitch += gx * dt;
    roll += gy * dt;
    yaw += gz * dt;

    float cy = cos(yaw * 0.5f);
    float sy = sin(yaw * 0.5f);
    float cp = cos(pitch * 0.5f);
    float sp = sin(pitch * 0.5f);
    float cr = cos(roll * 0.5f);
    float sr = sin(roll * 0.5f);

    quaternion->w = cr * cp * cy + sr * sp * sy;
    quaternion->x = sr * cp * cy - cr * sp * sy;
    quaternion->y = cr * sp * cy + sr * cp * sy;
    quaternion->z = cr * cp * sy - sr * sp * cy;

    return ESP_OK;
}

esp_err_t imu_calculate_euler_angles(const Quaternion *quaternion, EulerAngle *euler) {
    if (quaternion == NULL || euler == NULL) {
        return ESP_FAIL;
    }

    float sinr_cosp = 2 * (quaternion->w * quaternion->x + quaternion->y * quaternion->z);
    float cosr_cosp = 1 - 2 * (quaternion->x * quaternion->x + quaternion->y * quaternion->y);
    euler->roll = atan2(sinr_cosp, cosr_cosp);

    float sinp = 2 * (quaternion->w * quaternion->y - quaternion->z * quaternion->x);
    if (fabs(sinp) >= 1)
        euler->pitch = copysign(M_PI / 2, sinp);
    else
        euler->pitch = asin(sinp);

    float siny_cosp = 2 * (quaternion->w * quaternion->z + quaternion->x * quaternion->y);
    float cosy_cosp = 1 - 2 * (quaternion->y * quaternion->y + quaternion->z * quaternion->z);
    euler->yaw = atan2(siny_cosp, cosy_cosp);

    return ESP_OK;
}

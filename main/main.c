#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"


#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_GYRO_XOUT_H  0x43
#define MPU6050_PWR_MGMT_1   0x6B

#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0

#define GYRO_SCALE 131.0f 
#define DEG_TO_RAD (3.14159265358979323846 / 180.0f)

static uint8_t imu_dev_addr;

typedef struct {
    float x;
    float y;
    float z;
} AccelerationData;

typedef struct {
    float x;
    float y;
    float z;
} GyroscopeData;

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


esp_err_t imu_read_data(uint8_t reg_addr, uint8_t *data, size_t length) {
    if (data == NULL || length == 0) {
        return ESP_FAIL;
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (imu_dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_start(cmd);  
    i2c_master_write_byte(cmd, (imu_dev_addr << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, length, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);

    return err;
}

esp_err_t imu_init(uint8_t devAddr, gpio_num_t sda_pin, gpio_num_t scl_pin) {
    imu_dev_addr = devAddr;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda_pin,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = scl_pin,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) {
        return ESP_FAIL;
    }

    err = i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
    if (err != ESP_OK) {
        return ESP_FAIL;
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (imu_dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, MPU6050_PWR_MGMT_1, true);
    i2c_master_write_byte(cmd, 0x00, true);
    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);

    if (err == ESP_OK) {
        return ESP_OK;
    } else {
        return ESP_ERR_NOT_FOUND;
    }
}

esp_err_t imu_get_acceleration_data(AccelerationData *data) {
    if (data == NULL) {
        return ESP_FAIL;
    }

    uint8_t accel_data[6];
    esp_err_t err = imu_read_data(MPU6050_ACCEL_XOUT_H, accel_data, sizeof(accel_data));
    if (err != ESP_OK) {
        return err;
    }

    data->x = (int16_t)((accel_data[0] << 8) | accel_data[1]);
    data->y = (int16_t)((accel_data[2] << 8) | accel_data[3]);
    data->z = (int16_t)((accel_data[4] << 8) | accel_data[5]);

    return ESP_OK;
}

esp_err_t imu_get_gyroscope_data(GyroscopeData *data) {
    if (data == NULL) {
        return ESP_FAIL;
    }

    uint8_t gyro_data[6];
    esp_err_t err = imu_read_data(MPU6050_GYRO_XOUT_H, gyro_data, sizeof(gyro_data));
    if (err != ESP_OK) {
        return err;
    }

    data->x = (int16_t)((gyro_data[0] << 8) | gyro_data[1]);
    data->y = (int16_t)((gyro_data[2] << 8) | gyro_data[3]);
    data->z = (int16_t)((gyro_data[4] << 8) | gyro_data[5]);

    return ESP_OK;
}

esp_err_t imu_deinit() {
    esp_err_t err = i2c_driver_delete(I2C_MASTER_NUM);
    if (err != ESP_OK) {
        return ESP_FAIL;
    }
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

void app_main(void)
{
    esp_err_t resultado = imu_init(0x68, GPIO_NUM_21, GPIO_NUM_22);
    if (resultado != ESP_OK)
    {
        printf("Falha na inicialização do IMU.");
    }

    while (1)
    {
        printf("*******************\n");
        AccelerationData accel_data;
        resultado = imu_get_acceleration_data(&accel_data);
        if (resultado == ESP_OK)
        {
            printf("Aceleração: x = %f, y = %f, z = %f\n", accel_data.x, accel_data.y, accel_data.z);
        } else
        {
            printf("Falha ao obter dados de aceleração.\n");
        }
        

        GyroscopeData gyro_data;
        resultado = imu_get_gyroscope_data(&gyro_data);
        if (resultado == ESP_OK)
        {
            float gyro_x_rad_s = (gyro_data.x / GYRO_SCALE);
            float gyro_y_rad_s = (gyro_data.y / GYRO_SCALE);
            float gyro_z_rad_s = (gyro_data.z / GYRO_SCALE);
            printf("Giroscópio: x = %f, y = %f, z = %f\n", gyro_x_rad_s, gyro_y_rad_s, gyro_z_rad_s);
        } else {
            printf("Falha ao obter dados do giroscópio.\n");
        }

        IMUData imu_data = {
            .acel_x = accel_data.x,
            .acel_y = accel_data.y,
            .acel_z = accel_data.z,
            .giro_x = gyro_data.x,
            .giro_y = gyro_data.y,
            .giro_z = gyro_data.z,
        };

        Quaternion quaternion;
        resultado = imu_calculate_quaternion(&imu_data, &quaternion);
        if (resultado == ESP_OK)
        {
            printf("Quaternion: w = %f, x = %f, y = %f, z = %f\n", quaternion.w, quaternion.x, quaternion.y, quaternion.z);
        }
        else
        {
            printf("Falha ao calcular o Quaternion.\n");
        }

        EulerAngle euler_angle;
        resultado = imu_calculate_euler_angles(&quaternion, &euler_angle);
        if (resultado == ESP_OK)
        {
            printf("Ângulos de Euler: Roll = %f, Pitch = %f, Yaw = %f\n", euler_angle.roll, euler_angle.pitch, euler_angle.yaw);
        }
        else
        {
            printf("Falha ao calcular os ângulos de Euler.\n");
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}


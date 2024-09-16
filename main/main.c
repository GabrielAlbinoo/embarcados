#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "imu_tools.h"
#include "sensor_imu.h"

#define GYRO_SCALE 131.0f 
#define DEG_TO_RAD (3.14159265358979323846 / 180.0f)
#define GRAVITY 9.81f
#define ACCEL_SCALE 16384


void app_main(void)
{
    esp_err_t resultado = imu_init(0x68, GPIO_NUM_21, GPIO_NUM_22);
    if (resultado != ESP_OK)
    {
        printf("Falha na inicialização do IMU.");
        return;
    }

    while (1)
    {
        printf("*******************\n");
        AccelerationData accel_data;
        resultado = imu_get_acceleration_data(&accel_data);
        if (resultado == ESP_OK)
        {
            float acell_x = (accel_data.x/ACCEL_SCALE) * GRAVITY;
            float acell_y = (accel_data.y/ACCEL_SCALE) * GRAVITY;
            float acell_z = (accel_data.z/ACCEL_SCALE) * GRAVITY;
            printf("Aceleração (m/s): x = %f, y = %f, z = %f\n", acell_x, acell_y, acell_z);
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
            printf("Giroscópio (rad): x = %f, y = %f, z = %f\n", gyro_x_rad_s, gyro_y_rad_s, gyro_z_rad_s);
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


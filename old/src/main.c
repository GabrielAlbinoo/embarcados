#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sensor_imu.h"
#include "imu_tools.h"
#include "esp_log.h"

static const char *TAG = "main";

void app_main(void)
{
    esp_err_t resultado = imu_init(0x68, GPIO_NUM_21, GPIO_NUM_22);
    if (resultado == ESP_FAIL)
    {
        ESP_LOGE(TAG, "Falha na inicialização do IMU.");
        return;
    }

    AccelerationData accel_data;
    resultado = imu_get_acceleration_data(&accel_data);
    if (resultado == ESP_FAIL)
    {
        ESP_LOGE(TAG, "Falha ao obter dados de aceleração.");
    }

    GyroscopeData gyro_data;
    resultado = imu_get_gyroscope_data(&gyro_data);
    if (resultado == ESP_FAIL)
    {
        ESP_LOGE(TAG, "Falha ao obter dados do giroscópio.");
    }

    resultado = imu_deinit();
    if (resultado == ESP_FAIL)
    {
        ESP_LOGE(TAG, "Falha na desinicialização do IMU.");
    }

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

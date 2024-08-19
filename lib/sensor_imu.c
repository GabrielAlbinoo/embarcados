#include "sensor_imu.h"
#include "driver/i2c.h"

#define I2C_MASTER_NUM I2C_NUM_0          
#define I2C_MASTER_FREQ_HZ 100000          
#define I2C_MASTER_TX_BUF_DISABLE 0       
#define I2C_MASTER_RX_BUF_DISABLE 0       

static uint8_t imu_dev_addr;

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
    esp_err_t err = imu_read_data(MPU6050_ACCEL_XOUT_H, accel_data, 6);
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
    esp_err_t err = imu_read_data(MPU6050_GYRO_XOUT_H, gyro_data, 6);
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

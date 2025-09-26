#include "imu9dof.hpp"
#include "esp_err.h"
#include "esp_log.h"

static const char *TAG = "IMU9DoF";

namespace imu_sensor
{
IMU9DoF::IMU9DoF(i2c_port_t mpu6050_port, uint8_t mpu6050_address,
                 i2c_port_t qmc5883p_port, uint8_t qmc5883p_address)
{
    mpu6050_ = std::make_shared<MPU6050::MPU6050>(mpu6050_port, mpu6050_address);
    qmc5883p_ = std::make_shared<QMC5883P::QMC5883P>(qmc5883p_port, qmc5883p_address);
}

esp_err_t IMU9DoF::init()
{
    if (auto ret = mpu6050_->init(); ret != ESP_OK)
    {
        ESP_LOGE(TAG, "mpu6050 初始化失败");
        return ret;
    }

    if (auto ret = qmc5883p_->init(); ret != ESP_OK)
    {
        ESP_LOGE(TAG, "qmc5883p 初始化失败");
        return ret;
    }

    return ESP_OK;
}
}
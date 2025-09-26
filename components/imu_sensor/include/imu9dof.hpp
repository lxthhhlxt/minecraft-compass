#pragma once

#include "mpu6050.hpp"
#include "qmc5883p.hpp"

namespace imu_sensor
{
class IMU9DoF
{
public:
    IMU9DoF(i2c_port_t mpu6050_port, uint8_t mpu6050_address,
            i2c_port_t qmc5883p_port, uint8_t qmc5883p_address);

    ~IMU9DoF() = default;

    esp_err_t init();

private:
    std::shared_ptr<MPU6050::MPU6050> mpu6050_{nullptr};
    std::shared_ptr<QMC5883P::QMC5883P> qmc5883p_{nullptr};
};
}
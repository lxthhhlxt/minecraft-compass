#pragma once

#include "mpu6050.hpp"
#include "qmc5883p.hpp"

namespace imu_sensor
{
struct Data
{
    float accel_x{0.0f};
    float accel_y{0.0f};
    float accel_z{0.0f};

    float gyro_x{0.0f};
    float gyro_y{0.0f};
    float gyro_z{0.0f};

    float mag_x{0.0f};
    float mag_y{0.0f};
    float mag_z{0.0f};
};


// 定义四元数结构体
struct Quaternion
{
    double w;
    double x;
    double y;
    double z;
};

// 定义欧拉角结构体（弧度）
struct EulerAngles
{
    float roll;
    float pitch;
    float yaw;
};

class IMU9DoF
{
public:
    IMU9DoF(i2c_port_t mpu6050_port, uint8_t mpu6050_address,
            i2c_port_t qmc5883p_port, uint8_t qmc5883p_address);

    ~IMU9DoF() = default;

    esp_err_t init();

    void calc9AsixData();

private:
    std::shared_ptr<MPU6050::MPU6050> mpu6050_{nullptr};
    std::shared_ptr<QMC5883P::QMC5883P> qmc5883p_{nullptr};

};

void imuTask(void *Params);
}
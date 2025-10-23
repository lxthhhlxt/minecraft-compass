#pragma once

#include <driver/i2c.h>
#include "esp_err.h"
#include <memory>

namespace MPU6050
{
struct Data
{
    float accel_x{0.0f};
    float accel_y{0.0f};
    float accel_z{0.0f};
    float gyro_x{0.0f};
    float gyro_y{0.0f};
    float gyro_z{0.0f};

    float temp{0.0f};
};

struct Offset
{
    float accel_offset_x{0.0f};
    float accel_offset_y{0.0f};
    float accel_offset_z{0.0f};

    float gyro_offset_x{0.0f};
    float gyro_offset_y{0.0f};
    float gyro_offset_z{0.0f};
};

class MPU6050
{
public:
    MPU6050(const i2c_port_t port, const uint8_t address);

    esp_err_t init();

    bool calibrate();

    std::optional<std::shared_ptr<Data>> get_data();

    std::optional<std::shared_ptr<Data>> get_cal_data();

private:
    void saveOffsets(Offset& offset);

    std::optional<Offset> loadOffsets();

    esp_err_t read_byte(uint8_t reg, uint8_t* data);

    esp_err_t write_byte(uint8_t reg, uint8_t data);

    esp_err_t read_bytes(uint8_t reg, uint8_t* data, size_t length);

private:
    i2c_port_t i2c_port_;
    uint8_t dev_addr_;

    Offset offset_;
};
} // namespace MPU6050
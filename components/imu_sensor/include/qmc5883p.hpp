#pragma once
#include <driver/i2c.h>
#include "esp_err.h"
#include <memory>

namespace QMC5883P
{
struct Data
{
    float x{0.0f};
    float y{0.0f};
    float z{0.0f};
};

struct CalibrationParam
{
    float offset_x{0.0f};
    float offset_y{0.0f};
    float offset_z{0.0f};

    float scale_x{1.0f};
    float scale_y{1.0f};
    float scale_z{1.0f};
};

class QMC5883P
{
public:
    QMC5883P(const i2c_port_t port, const uint8_t addr);

    ~QMC5883P() = default;

    esp_err_t init();

    bool calibrate();

    std::optional<std::shared_ptr<Data>> get_data();

    std::optional<std::shared_ptr<Data>> get_cal_data();

private:
    void saveCalibrationParam(CalibrationParam& param);

    std::optional<CalibrationParam> loadCalibrationParam();

    esp_err_t read_bytes(uint8_t reg, uint8_t* data, size_t length);

    esp_err_t write_byte(uint8_t reg, uint8_t data);

private:
    i2c_port_t i2c_port_;
    uint8_t dev_addr_;

    CalibrationParam calibration_param_;
};
} // namespace QMC5883P
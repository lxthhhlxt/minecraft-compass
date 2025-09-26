#include "qmc5883p.hpp"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "freertos/projdefs.h"
#include "math.h"
#include "nvs_flash.h"
#include "nvs.h"

static const char *TAG = "QMC5883P";

constexpr uint16_t SAMPLE_NUM = 500;

namespace QMC5883P
{
QMC5883P::QMC5883P(const i2c_port_t port, const uint8_t addr)
    : i2c_port_(port), dev_addr_(addr)
{}

esp_err_t QMC5883P::init()
{
    // 设置坐标轴方向
    if (esp_err_t ret = write_byte(0x29, 0x06); ret != ESP_OK)
    {
        ESP_LOGE(TAG, "初始化坐标轴方向失败， err: 0x%x", ret);
        return ret;
    }

    // 满量程: ±8G
    if (esp_err_t ret = write_byte(0x0B, 0x08); ret != ESP_OK)
    {
        ESP_LOGE(TAG, "初始化量程失败， err: 0x%x", ret);
        return ret;
    }

    // 模式: Normal
    // 频率: 200Hz
    if (esp_err_t ret = write_byte(0x0A, 0xCD); ret != ESP_OK)
    {
        ESP_LOGE(TAG, "初始化模式&频率失败， err: 0x%x", ret);
        return ret;
    }

    if (auto calibration_param_opt = loadCalibrationParam(); calibration_param_opt == std::nullopt)
    {
        ESP_LOGE(TAG, "加载校准数据失败");
        calibrate();
    }
    else
    {
        calibration_param_ = calibration_param_opt.value();
        ESP_LOGI(TAG, "加载校准数据成功 offset:%f, %f, %f, scale:%f, %f, %f",
                      calibration_param_.offset_x, calibration_param_.offset_y, calibration_param_.offset_z,
                      calibration_param_.scale_x, calibration_param_.scale_y, calibration_param_.scale_z);
    }

    return ESP_OK;
}

bool QMC5883P::calibrate()
{
    ESP_LOGI(TAG, "=== 磁力计校准模式 ===");
    ESP_LOGI(TAG, "请缓慢地在空中画'8'字，持续约60秒");
    vTaskDelay(pdMS_TO_TICKS(2000));

    float x_min{32767.0f};
    float y_min{32767.0f};
    float z_min{32767.0f};
    float x_max{-32768.0f};
    float y_max{-32768.0f};
    float z_max{-32768.0f};

    for (int i = 0; i < SAMPLE_NUM; i++)
    {
        // 读取QMC5883P数据
        if (auto data_opt = get_data(); data_opt != std::nullopt)
        {
            auto data = data_opt.value();
            x_min = std::min(x_min, data->x);
            y_min = std::min(y_min, data->y);
            z_min = std::min(z_min, data->z);
            x_max = std::max(x_max, data->x);
            y_max = std::max(y_max, data->y);
            z_max = std::max(z_max, data->z);

            if (i % (SAMPLE_NUM / 10) == 0)
            {
                ESP_LOGI(TAG, "正在校准中 %.2f%%, x(%.2f - %.2f) y(%.2f - %.2f) z(%.2f - %.2f)",
                              (i / (float)SAMPLE_NUM) * 100,
                              x_min ,x_max,
                              y_min ,y_max,
                              z_min ,z_max);
            }
        }
        else
        {
            ESP_LOGE(TAG, "校准失败");
            return false;
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }

    // 计算偏移量 (硬铁干扰)
    calibration_param_.offset_x = (x_max + x_min) / 2.0f;
    calibration_param_.offset_y = (y_max + y_min) / 2.0f;
    calibration_param_.offset_z = (z_max + z_min) / 2.0f;

    // 计算缩放比例 (软铁干扰和灵敏度差异)
    float avg_delta = ((x_max - x_min) / 2.0f +
                       (y_max - y_min) / 2.0f +
                       (z_max - z_min) / 2.0f) / 3.0f;

    calibration_param_.scale_x = avg_delta / (x_max - x_min);
    calibration_param_.scale_y = avg_delta / (y_max - y_min);
    calibration_param_.scale_z = avg_delta / (z_max - z_min);

    ESP_LOGE(TAG, "校准成功");
    ESP_LOGI(TAG, "偏移量: X=%.2f, Y=%.2f, Z=%.2f",
                  calibration_param_.offset_x, calibration_param_.offset_y, calibration_param_.offset_z);
    ESP_LOGI(TAG, "缩放因子: X=%.2f, Y=%.2f, Z=%.2f",
                  calibration_param_.scale_x, calibration_param_.scale_y, calibration_param_.scale_z);

    saveCalibrationParam(calibration_param_);
    return true;
}

std::optional<std::shared_ptr<Data>> QMC5883P::get_data()
{
    uint8_t buffer[7];
    esp_err_t ret = read_bytes(0x00, buffer, 7);
    if (ret != ESP_OK) {
        return std::nullopt;
    }

    auto data = std::make_shared<Data>();
    data->x = (int16_t)((buffer[2] << 8) | buffer[1]) / 3750.0f;
    data->y = (int16_t)((buffer[4] << 8) | buffer[3]) / 3750.0f;
    data->z = (int16_t)((buffer[6] << 8) | buffer[5]) / 3750.0f;
    // ESP_LOGI(TAG, "chipID=%X X=%.2f, Y=%.2f, Z=%.2f", buffer[0], data->x, data->y, data->z);

    return data;
}

std::optional<std::shared_ptr<Data>> QMC5883P::get_cal_data()
{
    auto raw_data_opt = get_data();
    if (raw_data_opt == std::nullopt)
    {
        return std::nullopt;
    }
    auto raw_data = raw_data_opt.value();

    auto data = std::make_shared<Data>();
    data->x = (raw_data->x - calibration_param_.offset_x) * calibration_param_.scale_x;
    data->y = (raw_data->y - calibration_param_.offset_y) * calibration_param_.scale_y;
    data->z = (raw_data->z - calibration_param_.offset_z) * calibration_param_.scale_z;

    // float magnetic_field_strength = sqrtf(data->x * data->x +
    //                                       data->y * data->y +
    //                                       data->z * data->z);
    // ESP_LOGI(TAG, "校准:X=%f, Y=%f, Z=%f, 磁场总强度: %f\n", data->x, data->y, data->z, magnetic_field_strength);
    return data;
}

void QMC5883P::saveCalibrationParam(CalibrationParam& param)
{
    nvs_handle_t nvs_handle;
    ESP_ERROR_CHECK(nvs_open("qmc5883p_cal", NVS_READWRITE, &nvs_handle));

    ESP_ERROR_CHECK(nvs_set_blob(nvs_handle, "cal_param", &param, size_t(sizeof(CalibrationParam))));

    ESP_ERROR_CHECK(nvs_commit(nvs_handle));
    nvs_close(nvs_handle);
}

std::optional<CalibrationParam> QMC5883P::loadCalibrationParam()
{
    nvs_handle_t nvs_handle;
    if (nvs_open("qmc5883p_cal", NVS_READONLY, &nvs_handle) != ESP_OK)
    {
        ESP_LOGW(TAG, "NVS namespace qmc5883p_cal_param not found.");
        nvs_close(nvs_handle);
        return std::nullopt;
    }

    CalibrationParam cal_param;
    size_t length = sizeof(CalibrationParam);
    if (nvs_get_blob(nvs_handle, "cal_param", &cal_param, &length) != ESP_OK)
    {
        ESP_LOGW(TAG, "NVS key cal_param not found.");
        nvs_close(nvs_handle);
        return std::nullopt;
    }

    nvs_close(nvs_handle);
    return cal_param;
}

esp_err_t QMC5883P::read_bytes(uint8_t reg, uint8_t* data, size_t length) {
    if (length == 0) {
        return ESP_OK;
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr_ << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr_ << 1) | I2C_MASTER_READ, true);
    if (length > 1) {
        i2c_master_read(cmd, data, length - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + length - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_port_, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t QMC5883P::write_byte(uint8_t reg, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr_ << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_port_, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret;
}
}
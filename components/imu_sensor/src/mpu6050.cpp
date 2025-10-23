#include "mpu6050.hpp"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "freertos/projdefs.h"
#include "nvs_flash.h"
#include "nvs.h"
#include <cstddef>
#include "status_manager.hpp"

static const char *TAG = "MPU6050";

constexpr uint16_t SAMPLE_NUM = 1000;

namespace MPU6050
{
MPU6050::MPU6050(const i2c_port_t port, const uint8_t address)
    : i2c_port_(port), dev_addr_(address)
{}

esp_err_t MPU6050::init()
{
    // 唤醒设备
    if (esp_err_t ret = write_byte(0x6B, 0x00); ret != ESP_OK)
    {
        ESP_LOGE(TAG, "唤醒设备失败");
        return ret;
    }

    // 设置加速度计范围 ±4g
    if (esp_err_t ret = write_byte(0x1C, 0x08); ret != ESP_OK)
    {
        ESP_LOGE(TAG, "设置加速度计范围失败");
        return ret;
    }

    // 设置陀螺仪范围 ±500°/s
    if (esp_err_t ret = write_byte(0x1B, 0x08); ret != ESP_OK)
    {
        ESP_LOGE(TAG, "设置陀螺仪范围失败");
        return ret;
    }

    // 设置数字低通滤波器
    if (esp_err_t ret = write_byte(0x1A, 0x03); ret != ESP_OK)
    {
        ESP_LOGE(TAG, "设置低通滤波器失败");
        return ret;
    }

    // 使能AUX_I2C
    if (esp_err_t ret = write_byte(0x37, 0x02); ret != ESP_OK)
    {
        ESP_LOGE(TAG, "配置AUX_I2C失败");
        return ret;
    }

    // 设置采样率
    if (esp_err_t ret = write_byte(0x19, 0x04); ret != ESP_OK)
    {
        ESP_LOGE(TAG, "设置采样率失败");
        return ret;
    }

    if (auto offset_opt = loadOffsets(); offset_opt == std::nullopt)
    {
        ESP_LOGE(TAG, "加载校准数据失败");
        calibrate();
    }
    else
    {
        offset_ = offset_opt.value();
        ESP_LOGI(TAG, "加载校准数据成功 accel_offset:%f, %f, %f, gyro_offset:%f, %f, %f",
                      offset_.accel_offset_x, offset_.accel_offset_y, offset_.accel_offset_z,
                      offset_.gyro_offset_x, offset_.gyro_offset_y, offset_.gyro_offset_z);
        StatusManager::getInstance().setMPU6050Status(SensorStatus::READY);
    }

    return ESP_OK;
}

bool MPU6050::calibrate()
{
    StatusManager::getInstance().setMPU6050Status(SensorStatus::CALIBRATING);

    ESP_LOGI(TAG, "准备校准加速度计和陀螺仪...");
    vTaskDelay(pdMS_TO_TICKS(2000));

    float sum_accel_x = 0, sum_accel_y = 0, sum_accel_z = 0;
    float sum_gyro_x = 0, sum_gyro_y = 0, sum_gyro_z = 0;

    for (int i = 0; i < SAMPLE_NUM; i++)
    {
        if (auto raw_data_opt = get_data(); raw_data_opt != std::nullopt)
        {
            ESP_LOGI(TAG, "校准 %d", i);

            auto raw_data = raw_data_opt.value();
            sum_accel_x += raw_data->accel_x;
            sum_accel_y += raw_data->accel_y;
            sum_accel_z += raw_data->accel_z;

            sum_gyro_x += raw_data->gyro_x;
            sum_gyro_y += raw_data->gyro_y;
            sum_gyro_z += raw_data->gyro_z;

            vTaskDelay(pdMS_TO_TICKS(5));
        }
        else
        {
            ESP_LOGE(TAG, "校准失败");
            StatusManager::getInstance().setMPU6050Status(SensorStatus::NOT_INIT);
            return false;
        }
    }

    offset_.accel_offset_x = sum_accel_x / SAMPLE_NUM;
    offset_.accel_offset_y = sum_accel_y / SAMPLE_NUM;
    offset_.accel_offset_z = (sum_accel_z / SAMPLE_NUM) - 1.0f;

    offset_.gyro_offset_x = sum_gyro_x / SAMPLE_NUM;
    offset_.gyro_offset_y = sum_gyro_y / SAMPLE_NUM;
    offset_.gyro_offset_z = sum_gyro_z / SAMPLE_NUM;

    ESP_LOGI(TAG, "校准成功");
    ESP_LOGI(TAG, "加速度计偏置: %f, %f, %f", offset_.accel_offset_x, offset_.accel_offset_y, offset_.accel_offset_z);
    ESP_LOGI(TAG, "陀螺仪偏置: %f, %f, %f", offset_.gyro_offset_x, offset_.gyro_offset_y, offset_.gyro_offset_z);

    saveOffsets(offset_);

    StatusManager::getInstance().setMPU6050Status(SensorStatus::READY);
    return true;
}

std::optional<std::shared_ptr<Data>> MPU6050::get_data()
{
    uint8_t buffer[14];
    esp_err_t ret = read_bytes(0x3B, buffer, 14);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "读取数据失败");
        return std::nullopt;
    }

    auto data = std::make_shared<Data>();
    // 转换加速度数据
    data->accel_x = -(int16_t)((buffer[0] << 8) | buffer[1]) / 8192.0;
    data->accel_y = -(int16_t)((buffer[2] << 8) | buffer[3]) / 8192.0;
    data->accel_z = -(int16_t)((buffer[4] << 8) | buffer[5]) / 8192.0;
    // ESP_LOGI(TAG, "加速度: X=%.2f, Y=%.2f, Z=%.2f",
    //          data->accel_x, data->accel_y, data->accel_z);

    // 转换温度数据
    int16_t temp_raw = (int16_t)((buffer[6] << 8) | buffer[7]);
    data->temp = temp_raw / 340.0 + 36.53;

    // 转换陀螺仪数据
    data->gyro_x = (int16_t)((buffer[8]  << 8) | buffer[9])  / 65.5;
    data->gyro_y = (int16_t)((buffer[10] << 8) | buffer[11]) / 65.5;
    data->gyro_z = (int16_t)((buffer[12] << 8) | buffer[13]) / 65.5;
    // ESP_LOGI(TAG, "陀螺仪: X=%.2f, Y=%.2f, Z=%.2f",
    //          data->gyro_x, data->gyro_y, data->gyro_z);

    return data;
}

std::optional<std::shared_ptr<Data>> MPU6050::get_cal_data()
{
    auto raw_data_opt = get_data();
    if (raw_data_opt == std::nullopt)
    {
        return std::nullopt;
    }
    auto raw_data = raw_data_opt.value();

    auto cal_data = std::make_shared<Data>();
    cal_data->accel_x = raw_data->accel_x - offset_.accel_offset_x;
    cal_data->accel_y = raw_data->accel_y - offset_.accel_offset_x;
    cal_data->accel_z = raw_data->accel_z - offset_.accel_offset_x;
    cal_data->gyro_x  = raw_data->gyro_x  - offset_.accel_offset_x;
    cal_data->gyro_y  = raw_data->gyro_y  - offset_.accel_offset_x;
    cal_data->gyro_z  = raw_data->gyro_z  - offset_.accel_offset_x;

    return cal_data;
}

void MPU6050::saveOffsets(Offset& offset)
{
    nvs_handle_t nvs_handle;
    ESP_ERROR_CHECK(nvs_open("mpu6050_offsets", NVS_READWRITE, &nvs_handle));

    ESP_ERROR_CHECK(nvs_set_blob(nvs_handle, "offset", &offset, size_t(sizeof(Offset))));

    ESP_ERROR_CHECK(nvs_commit(nvs_handle));
    nvs_close(nvs_handle);
}

std::optional<Offset> MPU6050::loadOffsets()
{
    nvs_handle_t nvs_handle;
    if (nvs_open("mpu6050_offsets", NVS_READONLY, &nvs_handle) != ESP_OK)
    {
        ESP_LOGW(TAG, "NVS namespace mpu6050_offsets not found.");
        nvs_close(nvs_handle);
        return std::nullopt;
    }

    Offset offset;
    size_t length = sizeof(Offset);
    if (nvs_get_blob(nvs_handle, "offset", &offset, &length) != ESP_OK)
    {
        ESP_LOGW(TAG, "NVS key offset not found.");
        nvs_close(nvs_handle);
        return std::nullopt;
    }

    nvs_close(nvs_handle);
    return offset;
}

esp_err_t MPU6050::read_byte(uint8_t reg, uint8_t* data) {
    return read_bytes(reg, data, 1);
}

esp_err_t MPU6050::write_byte(uint8_t reg, uint8_t data) {
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

esp_err_t MPU6050::read_bytes(uint8_t reg, uint8_t* data, size_t length) {
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
}
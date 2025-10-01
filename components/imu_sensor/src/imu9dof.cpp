#include "imu9dof.hpp"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <cmath>

static const char *TAG = "IMU9DoF";

// 定义I2C配置
constexpr gpio_num_t mpu6050_scl_io   = GPIO_NUM_4;
constexpr gpio_num_t mpu6050_sda_io   = GPIO_NUM_5;
constexpr uint32_t   mpu6050_freq_hz  = 400000;
constexpr i2c_port_t mpu6050_i2c_num  = I2C_NUM_0;
constexpr uint8_t    mpu6050_addr     = 0x68;
constexpr i2c_port_t qmc5883p_i2c_num = I2C_NUM_0;
constexpr uint8_t    qmc5883p_addr    = 0x2C;
constexpr uint16_t   imu_freq         = 100;

// 初始化I2C主机
static void i2c_master_init()
{
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = mpu6050_sda_io;
    conf.scl_io_num = mpu6050_scl_io;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = mpu6050_freq_hz;
    conf.clk_flags = 0;

    esp_err_t err = i2c_param_config(mpu6050_i2c_num, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C参数配置失败: %d", err);
        return;
    }

    err = i2c_driver_install(mpu6050_i2c_num, conf.mode, 0, 0, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C驱动安装失败: %d", err);
    }
}

// I2C扫描函数
void i2c_scanner()
{
    ESP_LOGI(TAG, "Scanning I2C bus...\n");
    for (uint8_t addr = 1; addr < 127; addr++)
    {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(mpu6050_i2c_num, cmd, pdMS_TO_TICKS(100));
        i2c_cmd_link_delete(cmd);

        if (ret == ESP_OK)
        {
            ESP_LOGI(TAG, "Device found at address 0x%02X\n", addr);
        }
    }
    ESP_LOGI(TAG, "Scan completed.\n");
}

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

void IMU9DoF::calc9AsixData()
{

}

void imuTask(void *Params)
{
    i2c_master_init();

    i2c_scanner();

    imu_sensor::IMU9DoF imu(mpu6050_i2c_num,  mpu6050_addr,
                            qmc5883p_i2c_num, qmc5883p_addr);
    imu.init();

    while (1)
    {
        int32_t start_time = esp_timer_get_time() / 1000;
        imu.calc9AsixData();
        int32_t end_time = esp_timer_get_time() / 1000;
        int32_t sleep_time = (1000 / imu_freq) - std::max(int32_t(0), (end_time - start_time));
        vTaskDelay(pdMS_TO_TICKS(sleep_time));
    }
}
}
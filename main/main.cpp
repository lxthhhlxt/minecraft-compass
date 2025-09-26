#include <stdio.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "hal/uart_types.h"
#include "imu9dof.hpp"
#include "nvs_flash.h"
#include "nvs.h"
#include "soc/gpio_num.h"
#include "driver/uart.h"
#include "GTU8.hpp"

static const char *TAG = "MineCraft-Compass";

// 定义I2C配置
constexpr gpio_num_t i2c_master_scl_io  = GPIO_NUM_4;
constexpr gpio_num_t i2c_master_sda_io  = GPIO_NUM_5;
constexpr uint32_t   i2c_master_freq_hz = 400000;
constexpr i2c_port_t mpu6050_i2c_num    = I2C_NUM_0;
constexpr uint8_t    mpu6050_addr       = 0x68;
constexpr i2c_port_t qmc5883p_i2c_num   = I2C_NUM_0;
constexpr uint8_t    qmc5883p_addr      = 0x2C;

// 定义UART配置
constexpr uart_port_t uart_port_num       = UART_NUM_1;  // 使用的UART端口号
constexpr uint32_t    uart_baud_rate      = 9600;        // 串口设备的波特率
constexpr gpio_num_t  uart_rx_pin         = GPIO_NUM_18; // ESP32的RX引脚（接设备的TX）
constexpr gpio_num_t  uart_tx_pin         = GPIO_NUM_17; // ESP32的TX引脚（接设备的RX）
constexpr uint32_t    uart_rx_buffer_size = 1024;        // RX缓冲区大小
constexpr uint32_t    uart_tx_buffer_size = 0;           // TX缓冲区大小（若不发送可设为0）
constexpr uint32_t    uart_queue_size     = 10;          // 事件队列大小

void nvs_init()
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
}

// 初始化I2C主机
static void i2c_master_init() {
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = i2c_master_sda_io;
    conf.scl_io_num = i2c_master_scl_io;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = i2c_master_freq_hz;
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
void i2c_scanner() {
    ESP_LOGI(TAG, "Scanning I2C bus...\n");
    for (uint8_t addr = 1; addr < 127; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(mpu6050_i2c_num, cmd, pdMS_TO_TICKS(100));
        i2c_cmd_link_delete(cmd);

        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Device found at address 0x%02X\n", addr);
        }
    }
    ESP_LOGI(TAG, "Scan completed.\n");
}

void setup_uart() {
    // 步骤1: 配置UART参数
    uart_config_t uart_config = {
        .baud_rate = uart_baud_rate,
        .data_bits = UART_DATA_8_BITS,   // 8位数据位:cite[2]:cite[3]:cite[6]
        .parity = UART_PARITY_DISABLE,   // 无校验:cite[2]:cite[3]:cite[6]
        .stop_bits = UART_STOP_BITS_1,   // 1位停止位:cite[2]:cite[3]:cite[6]
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, // 无硬件流控:cite[3]
        .source_clk = UART_SCLK_DEFAULT, // 默认时钟源:cite[3]
    };
    ESP_ERROR_CHECK(uart_param_config(uart_port_num, &uart_config));

    // 步骤2: 设置UART引脚
    ESP_ERROR_CHECK(uart_set_pin(uart_port_num, uart_tx_pin, uart_rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // 步骤3: 安装UART驱动程序，设置缓冲区大小和事件队列
    ESP_ERROR_CHECK(uart_driver_install(uart_port_num, uart_rx_buffer_size, uart_tx_buffer_size, uart_queue_size, NULL, 0));
}

extern "C" void app_main()
{
    nvs_init();

    i2c_master_init();

    i2c_scanner();

    setup_uart();

    imu_sensor::IMU9DoF imu(mpu6050_i2c_num, mpu6050_addr,
                            qmc5883p_i2c_num, qmc5883p_addr);
    imu.init();

    auto gps = std::make_shared<gps_sensor::GTU8>(uart_port_num);

    xTaskCreate(
        [](void *Param) {
            auto gps_ptr = static_cast<std::shared_ptr<gps_sensor::GTU8> *>(Param);
            auto gps = *gps_ptr;
            gps->update();
        }, "gps_task", 4096, &gps, 5, NULL);
}
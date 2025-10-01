#include <cstddef>
#include <stdio.h>
#include "compressed_led.hpp"
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
    xTaskCreate(imu_sensor::imuTask,     "imu_task", 4096, NULL, 3, NULL);
    xTaskCreate(gps_sensor::gpsTask,     "gps_task", 4096, NULL, 2, NULL);
    xTaskCreate(compressed_led::ledTask, "led_task", 4096, NULL, 1, NULL);
}

// extern "C" void app_main()
// {
//     nvs_init();

//     i2c_master_init();

//     i2c_scanner();

//     setup_uart();

//     imu_sensor::IMU9DoF imu(mpu6050_i2c_num, mpu6050_addr,
//                             qmc5883p_i2c_num, qmc5883p_addr);
//     imu.init();

//     auto gps = std::make_shared<gps_sensor::GTU8>(uart_port_num);


//     xTaskCreate(
//         [](void *Param) {
//             auto gps_ptr = static_cast<std::shared_ptr<gps_sensor::GTU8> *>(Param);
//             auto gps = *gps_ptr;
//             gps->update();
//         }, "gps_task", 4096, &gps, 5, NULL);

//     while (1)
//     {
//         static angle = 0.0f;
//         auto gps_status = gps_sensor::GPSStatus::NOT_POSITIONED;
//         {
//             std::lock_guard<std::mutex> lock(gps->data_mutex_);
//             if (gps->data_)
//             {
//                 gps_status = gps->data_->gps_status_;
//             }
//         }

//         if (gps_status == gps_sensor::GPSStatus::NOT_POSITIONED || gps_status == gps_sensor::GPSStatus::UNKNOWN)
//         {
//             angle += 5.0f;
//         }
//
//     }
// }
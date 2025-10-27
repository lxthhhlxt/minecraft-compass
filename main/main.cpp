#include <cstddef>
#include <stdio.h>
#include "compressed_ble.hpp"
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

extern "C" void app_main()
{
    nvs_init();
    xTaskCreate(imu_sensor::imuTask,     "imu_task", 4096, NULL, 4, NULL);
    xTaskCreate(gps_sensor::gpsTask,     "gps_task", 4096, NULL, 3, NULL);
    // xTaskCreate(compressed_ble::bleTask, "ble_task", 4096, NULL, 2, NULL);
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
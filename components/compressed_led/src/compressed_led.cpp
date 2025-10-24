#include "compressed_led.hpp"
#include "status_manager.hpp"
#include "data_manager.hpp"
#include "esp_timer.h"
#include "esp_log.h"

// 硬件配置
constexpr gpio_num_t LED_STRIP_GPIO_NUM = GPIO_NUM_6;   // 连接WS2812 DIN的GPIO引脚
constexpr uint8_t LED_STRIP_LED_NUMBER = 56;            // 灯带上WS2812灯珠的数量
constexpr uint32_t LED_STRIP_RMT_RES_HZ = (10 * 1000 * 1000); // RMT分辨率：10MHz, 1 tick = 0.1us

// deep gary R G B paras
constexpr uint32_t DEEP_GRAY_R = 180;
constexpr uint32_t DEEP_GRAY_G = 220;
constexpr uint32_t DEEP_GRAY_B = 255;

// light gray R G B paras
constexpr uint32_t LIGHT_GRAY_R = 180;
constexpr uint32_t LIGHT_GRAY_G = 220;
constexpr uint32_t LIGHT_GRAY_B = 255;

// red R G B paras
constexpr uint32_t RED_R = 255;
constexpr uint32_t RED_G = 0;
constexpr uint32_t RED_B = 0;

namespace compressed_led
{
CompressedLed::CompressedLed()
{
    init();
}

void CompressedLed::setLedByStatus(DisplayState &display_state)
{
    static DisplayState latest_state{DisplayState::DISPLAY_DESTNATION};

    if (latest_state != display_state)
    {
        ESP_LOGI("LED", "DisplayState: %d\n", static_cast<int>(display_state));
        latest_state = display_state;
    }

    // led_strip_clear(led_strip_);
    switch (display_state)
    {
        case DisplayState::DISPLAY_NONE:
        {
            setNoneLed();
            break;
        }
        case DisplayState::DISPLAY_MPU6050_CALIBRATING:
        {
            set6050CalibratingLed();
            break;
        }
        case DisplayState::DISPLAY_QMC5883P_CALIBRATING:
        {
            set5883CalibratingLed();
            break;
        }
        case DisplayState::DISPLAY_GPS_CALIBRATING:
        {
            break;
        }
        case DisplayState::DISPLAY_COMPASS:
        {
            if (auto imu_data = DataManager::getInstance().getLatestIMUData(); imu_data)
            {
                setCompressedLedByAngle(-imu_data->yaw);
                // ESP_LOGI("LED", "Yaw=%.2f", -imu_data->yaw);
            }
            break;
        }
        case DisplayState::DISPLAY_DESTNATION:
        {
            break;
        }
    }
    led_strip_refresh(led_strip_);
}

void CompressedLed::init()
{
    // 1. 配置LED Strip
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_STRIP_GPIO_NUM,
        .max_leds = LED_STRIP_LED_NUMBER,
        .led_pixel_format = LED_PIXEL_FORMAT_GRB, // 注意颜色格式！
        .led_model = LED_MODEL_WS2812,
        .flags = {
            .invert_out = false,
        },
    };

    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = LED_STRIP_RMT_RES_HZ,
        .mem_block_symbols = 0,
        .flags = {
            .with_dma = false,
        },
    };

    // 2. 初始化LED Strip，并检查错误
    esp_err_t err = led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip_);
    if (err != ESP_OK) {
        // 打印错误信息
        return;
    }

    // 3. 清除可能存在的残留显示
    led_strip_clear(led_strip_);
    vTaskDelay(pdMS_TO_TICKS(500));

    // 5. 务必刷新才能显示！
    led_strip_refresh(led_strip_);
}

void CompressedLed::setCompressedLedByAngle(float angle)
{
    while (angle > 360.0 || angle < 0)
    {
        if (angle > 360.0)
        {
            angle -= 360.0;
        }
        else if (angle < 0)
        {
            angle += 360.0;
        }
    }

    if (angle >= 348.7 || angle < 11.3)
    {
        set_compressed_led(std::vector<uint8_t>{21, 34, 44}, // red
                           std::vector<uint8_t>{22, 11}, // deep gray
                           std::vector<uint8_t>{20}); // light gray
    }
    else if (angle >= 11.3 && angle < 31)
    {
        set_compressed_led(std::vector<uint8_t>{21, 34, 35, 43}, // red
                           std::vector<uint8_t>{11, 20}, // deep gray
                           std::vector<uint8_t>{22}); // light gray
    }
    else if (angle >= 31 && angle < 59.1)
    {
        set_compressed_led(std::vector<uint8_t>{21, 34, 35, 43, 42}, // red
                           std::vector<uint8_t>{11, 20}, // deep gray
                           std::vector<uint8_t>{10, 22}); // light gray
    }
    else if (angle >= 59.1 && angle < 66.9)
    {
        set_compressed_led(std::vector<uint8_t>{21, 20, 35, 36, 42, 41}, // red
                           std::vector<uint8_t>{10}, // deep gray
                           std::vector<uint8_t>{22}); // light gray
    }
    else if (angle >= 66.9 && angle < 71.7)
    {
        set_compressed_led(std::vector<uint8_t>{21, 20, 35, 36, 37}, // red
                           std::vector<uint8_t>{10}, // deep gray
                           std::vector<uint8_t>{9, 22}); // light gray
    }
    else if (angle >= 71.7 && angle < 82)
    {
        set_compressed_led(std::vector<uint8_t>{21, 20, 35, 36, 37, 38}, // red
                           std::vector<uint8_t>{10}, // deep gray
                           std::vector<uint8_t>{9, 22}); // light gray
    }
    else if (angle >= 82 && angle < 83.8)
    {
        set_compressed_led(std::vector<uint8_t>{21, 20, 19, 36, 37, 38}, // red
                           std::vector<uint8_t>{11, 23}, // deep gray
                           std::vector<uint8_t>{22, 34}); // light gray
    }
    else if (angle >= 83.8 && angle < 96.5)
    {
        set_compressed_led(std::vector<uint8_t>{21, 20, 19, 18, 17}, // red
                           std::vector<uint8_t>{11, 23}, // deep gray
                           std::vector<uint8_t>{22, 34}); // light gray
    }
    else if (angle >= 96.5 && angle < 113.4)
    {
        set_compressed_led(std::vector<uint8_t>{21, 20, 19, 13, 14}, // red
                           std::vector<uint8_t>{11, 23}, // deep gray
                           std::vector<uint8_t>{22, 34}); // light gray
    }
    else if (angle >= 113.4 && angle < 121.2)
    {
        set_compressed_led(std::vector<uint8_t>{21, 20, 12, 13, 14}, // red
                           std::vector<uint8_t>{32}, // deep gray
                           std::vector<uint8_t>{22, 33}); // light gray
    }
    else if (angle >= 121.2 && angle < 135.2)
    {
        set_compressed_led(std::vector<uint8_t>{21, 20, 12, 13, 0}, // red
                           std::vector<uint8_t>{}, // deep gray
                           std::vector<uint8_t>{22, 33}); // light gray
    }
    else if (angle >= 135.2 && angle < 149.2)
    {
        set_compressed_led(std::vector<uint8_t>{21, 11, 12, 1, 0}, // red
                           std::vector<uint8_t>{22, 20}, // deep gray
                           std::vector<uint8_t>{33, 34}); // light gray
    }
    else if (angle >= 149.2 && angle < 168.9)
    {
        set_compressed_led(std::vector<uint8_t>{21, 11, 12, 1}, // red
                           std::vector<uint8_t>{22, 20}, // deep gray
                           std::vector<uint8_t>{34}); // light gray
    }
    else if (angle >= 168.9 && angle < 191.5)
    {
        set_compressed_led(std::vector<uint8_t>{21, 11, 2}, // red
                           std::vector<uint8_t>{22, 20}, // deep gray
                           std::vector<uint8_t>{34}); // light gray
    }
    else if (angle >= 191.5 && angle < 211.2)
    {
        set_compressed_led(std::vector<uint8_t>{21, 10, 11, 3}, // red
                           std::vector<uint8_t>{22, 20}, // deep gray
                           std::vector<uint8_t>{34}); // light gray
    }
    else if (angle >= 211.2 && angle < 225.2)
    {
        set_compressed_led(std::vector<uint8_t>{21, 10, 11, 4, 3}, // red
                           std::vector<uint8_t>{22, 20}, // deep gray
                           std::vector<uint8_t>{34, 35}); // light gray
    }
    else if (angle >= 225.2 && angle < 234.7)
    {
        set_compressed_led(std::vector<uint8_t>{21, 22, 10, 9, 5, 4}, // red
                           std::vector<uint8_t>{35}, // deep gray
                           std::vector<uint8_t>{20}); // light gray
    }
    else if (angle >= 234.7 && angle < 247)
    {
        set_compressed_led(std::vector<uint8_t>{21, 22, 8, 9, 10, 5}, // red
                           std::vector<uint8_t>{20, 36}, // deep gray
                           std::vector<uint8_t>{35}); // light gray
    }
    else if (angle >= 247 && angle < 251.8)
    {
        set_compressed_led(std::vector<uint8_t>{21, 22, 23, 10, 9, 8, 7,}, // red
                           std::vector<uint8_t>{20, 36}, // deep gray
                           std::vector<uint8_t>{35}); // light gray
    }
    else if (angle >= 251.8 && angle < 265)
    {
        set_compressed_led(std::vector<uint8_t>{21, 22, 23, 9, 8, 7}, // red
                           std::vector<uint8_t>{19, 11}, // deep gray
                           std::vector<uint8_t>{20, 34}); // light gray
    }
    else if (angle >= 265 && angle < 275.4)
    {
        set_compressed_led(std::vector<uint8_t>{21, 22, 23, 24, 25, 26}, // red
                           std::vector<uint8_t>{11, 19}, // deep gray
                           std::vector<uint8_t>{20, 34}); // light gray
    }
    else if (angle >= 275.4 && angle < 288.6)
    {
        set_compressed_led(std::vector<uint8_t>{21, 22, 23, 32, 31, 30}, // red
                           std::vector<uint8_t>{11, 19}, // deep gray
                           std::vector<uint8_t>{20, 34}); // light gray
    }
    else if (angle >= 288.6 && angle < 293.4)
    {
        set_compressed_led(std::vector<uint8_t>{21, 22, 33, 32, 31, 30}, // red
                           std::vector<uint8_t>{12}, // deep gray
                           std::vector<uint8_t>{20, 13}); // light gray
    }
    else if (angle >= 293.4 && angle < 305.7)
    {
        set_compressed_led(std::vector<uint8_t>{21, 22, 33, 32, 31, 47}, // red
                           std::vector<uint8_t>{12}, // deep gray
                           std::vector<uint8_t>{13, 20}); // light gray
    }
    else if (angle >= 305.7 && angle < 315.2)
    {
        set_compressed_led(std::vector<uint8_t>{21, 22, 33, 32, 46, 47}, // red
                           std::vector<uint8_t>{12}, // deep gray
                           std::vector<uint8_t>{20}); // light gray
    }
    else if (angle >= 315.2 && angle < 329.2)
    {
        set_compressed_led(std::vector<uint8_t>{21, 34, 33, 45, 46}, // red
                           std::vector<uint8_t>{11, 22}, // deep gray
                           std::vector<uint8_t>{12, 20}); // light gray
    }
    else if (angle >= 329.2 && angle < 348.7)
    {
        set_compressed_led(std::vector<uint8_t>{21, 34, 33, 45}, // red
                           std::vector<uint8_t>{11, 22}, // deep gray
                           std::vector<uint8_t>{20}); // light gray
    }
}

void CompressedLed::set_leds_rgb(const std::vector<uint8_t>& led_nums, const uint32_t r, const uint32_t g, const uint32_t b)
{
    for (auto led_num : led_nums)
    {
        led_strip_set_pixel(led_strip_, led_num, r, g, b);
    }
}

void CompressedLed::set_compressed_led(const std::vector<uint8_t>& red_led_nums,
                                       const std::vector<uint8_t>& deep_gray_led_nums,
                                       const std::vector<uint8_t>& light_gray_led_nums)
{
    for (int led_num = 0; led_num < LED_STRIP_LED_NUMBER; led_num++)
    {
        led_strip_set_pixel(led_strip_, led_num, 0, 0, 0);
    }

    set_leds_rgb(red_led_nums, RED_R, RED_G, RED_B);
    set_leds_rgb(deep_gray_led_nums, DEEP_GRAY_R, DEEP_GRAY_G, DEEP_GRAY_B);
    set_leds_rgb(light_gray_led_nums, LIGHT_GRAY_R, LIGHT_GRAY_G, LIGHT_GRAY_B);
}

void CompressedLed::setNoneLed()
{
    led_strip_set_pixel(led_strip_, 0, 255, 0, 0);
}

void CompressedLed::set6050CalibratingLed()
{
    led_strip_set_pixel(led_strip_, 1, 0, 255, 0);
}

void CompressedLed::set5883CalibratingLed()
{
    led_strip_set_pixel(led_strip_, 2, 0, 0, 255);
}

void ledTask(void *Params)
{
    auto compressed_led = CompressedLed();
    auto display_state{DisplayState::DISPLAY_NONE};

    while (1)
    {
        int32_t start_time = esp_timer_get_time() / 1000;

        display_state = StatusManager::getInstance().getDisplayState();

        compressed_led.setLedByStatus(display_state);

        int32_t end_time = esp_timer_get_time() / 1000;
        int32_t sleep_time = 100 - std::max((end_time - start_time), int32_t(0));
        vTaskDelay(pdMS_TO_TICKS(sleep_time)); // 每1秒读取一次
    }
}
}
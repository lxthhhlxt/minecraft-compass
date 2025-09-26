#pragma once

#include <stdint.h>
#include "led_strip.h"
#include "soc/gpio_num.h"
#include <vector>

// 硬件配置
constexpr gpio_num_t LED_STRIP_GPIO_NUM = GPIO_NUM_6;   // 连接WS2812 DIN的GPIO引脚
constexpr uint8_t LED_STRIP_LED_NUMBER = 56;            // 灯带上WS2812灯珠的数量
constexpr uint32_t LED_STRIP_RMT_RES_HZ = (10 * 1000 * 1000); // RMT分辨率：10MHz, 1 tick = 0.1us

class CompressedLed
{
public:
    CompressedLed(led_strip_handle_t led_strip);

    ~CompressedLed() = default;

    void set_compressed_led_by_angle(float angle);

private:
    void set_leds_rgb(const std::vector<uint8_t>& led_nums, const uint32_t r, const uint32_t g, const uint32_t b);

    void set_compressed_led(const std::vector<uint8_t>& red_led_nums,
                            const std::vector<uint8_t>& deep_gray_led_nums,
                            const std::vector<uint8_t>& light_gray_led_nums);

private:
    led_strip_handle_t led_strip_;
};


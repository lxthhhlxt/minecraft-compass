#pragma once

#include <stdint.h>
#include "led_strip.h"
#include "soc/gpio_num.h"
#include <vector>

namespace compressed_led
{
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

void ledTask(void *Params);
}
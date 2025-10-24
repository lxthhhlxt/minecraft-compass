#pragma once

#include <stdint.h>
#include "led_strip.h"
#include "soc/gpio_num.h"
#include "status_manager.hpp"
#include <vector>

namespace compressed_led
{
class CompressedLed
{
public:
    CompressedLed();

    ~CompressedLed() = default;

    void setLedByStatus(DisplayState &display_state);

private:
    void init();

    void setCompressedLedByAngle(float angle);

    void set_leds_rgb(const std::vector<uint8_t>& led_nums, const uint32_t r, const uint32_t g, const uint32_t b);

    void set_compressed_led(const std::vector<uint8_t>& red_led_nums,
                            const std::vector<uint8_t>& deep_gray_led_nums,
                            const std::vector<uint8_t>& light_gray_led_nums);
    void setNoneLed();

    void set6050CalibratingLed();

    void set5883CalibratingLed();

private:
    led_strip_handle_t led_strip_;
};

void ledTask(void *Params);
}
#include "compressed_led.hpp"

// deep gary R G B paras
constexpr uint32_t DEEP_GRAY_R = 180;
constexpr uint32_t DEEP_GRAY_G = 220;
constexpr uint32_t DEEP_GRAY_B = 255;

// light gray R G B paras
constexpr uint32_t LIGHT_GRAY_R = 180;
constexpr uint32_t LIGHT_GRAY_G = 220;
constexpr uint32_t LIGHT_GRAY_B = 255;

// red R G B paras
constexpr uint32_t RED_R = 0;
constexpr uint32_t RED_G = 0;
constexpr uint32_t RED_B = 0;

CompressedLed::CompressedLed(led_strip_handle_t led_strip)
: led_strip_(led_strip)
{};

void CompressedLed::set_compressed_led_by_angle(float angle)
{
    while (angle > 360.0)
    {
        angle -= 360.0;
    }
    uint8_t zone = (angle + (360.0 / 27.0) / 2.0) / (360.0 / 27.0);

    switch (zone)
    {
        case 0:
        {
            set_compressed_led(std::vector<uint8_t>{21, 34, 44}, // red
                               std::vector<uint8_t>{22, 11}, // deep gray
                               std::vector<uint8_t>{20}); // light gray
            break;
        }
        case 1:
        {
            set_compressed_led(std::vector<uint8_t>{21, 34, 35, 43}, // red
                               std::vector<uint8_t>{11, 20}, // deep gray
                               std::vector<uint8_t>{22}); // light gray
            break;
        }
        case 2:
        {
            set_compressed_led(std::vector<uint8_t>{21, 34, 35, 43, 42}, // red
                               std::vector<uint8_t>{11, 20}, // deep gray
                               std::vector<uint8_t>{10, 22}); // light gray
            break;
        }
        case 3:
        {
            set_compressed_led(std::vector<uint8_t>{21, 20, 35, 36, 42, 41}, // red
                               std::vector<uint8_t>{10}, // deep gray
                               std::vector<uint8_t>{22}); // light gray
            break;
        }
        case 4:
        {
            set_compressed_led(std::vector<uint8_t>{21, 20, 35, 36, 37}, // red
                               std::vector<uint8_t>{10}, // deep gray
                               std::vector<uint8_t>{9, 22}); // light gray
            break;
        }
        case 5:
        {
            set_compressed_led(std::vector<uint8_t>{21, 20, 35, 36, 37, 38}, // red
                               std::vector<uint8_t>{10}, // deep gray
                               std::vector<uint8_t>{9, 22}); // light gray
            break;
        }
        case 6:
        {
            set_compressed_led(std::vector<uint8_t>{21, 20, 19, 36, 37, 38}, // red
                               std::vector<uint8_t>{11, 23}, // deep gray
                               std::vector<uint8_t>{22, 34}); // light gray
            break;
        }
        case 7:
        {
            set_compressed_led(std::vector<uint8_t>{21, 20, 19, 18, 17}, // red
                               std::vector<uint8_t>{11, 23}, // deep gray
                               std::vector<uint8_t>{22, 34}); // light gray
            break;
        }
        case 8:
        {
            set_compressed_led(std::vector<uint8_t>{21, 20, 19, 13, 14}, // red
                               std::vector<uint8_t>{11, 23}, // deep gray
                               std::vector<uint8_t>{22, 34}); // light gray
            break;
        }
        case 9:
        {
            set_compressed_led(std::vector<uint8_t>{21, 20, 12, 13, 14}, // red
                               std::vector<uint8_t>{32}, // deep gray
                               std::vector<uint8_t>{22, 33}); // light gray
            break;
        }
        case 10:
        {
            set_compressed_led(std::vector<uint8_t>{21, 20, 12, 13, 0}, // red
                               std::vector<uint8_t>{}, // deep gray
                               std::vector<uint8_t>{22, 33}); // light gray
            break;
        }
        case 11:
        {
            set_compressed_led(std::vector<uint8_t>{21, 11, 12, 1, 0}, // red
                               std::vector<uint8_t>{22, 20}, // deep gray
                               std::vector<uint8_t>{33, 34}); // light gray
            break;
        }
        case 12:
        {
            set_compressed_led(std::vector<uint8_t>{21, 11, 12, 1}, // red
                               std::vector<uint8_t>{22, 20}, // deep gray
                               std::vector<uint8_t>{34}); // light gray
            break;
        }
        case 13:
        {
            set_compressed_led(std::vector<uint8_t>{21, 11, 2}, // red
                               std::vector<uint8_t>{22, 20}, // deep gray
                               std::vector<uint8_t>{34}); // light gray
            break;
        }
        case 14:
        {
            set_compressed_led(std::vector<uint8_t>{21, 10, 11, 3}, // red
                               std::vector<uint8_t>{22, 20}, // deep gray
                               std::vector<uint8_t>{34}); // light gray
            break;
        }
        case 15:
        {
            set_compressed_led(std::vector<uint8_t>{21, 10, 11, 4, 3}, // red
                               std::vector<uint8_t>{22, 20}, // deep gray
                               std::vector<uint8_t>{34, 35}); // light gray
            break;
        }
        case 16:
        {
            set_compressed_led(std::vector<uint8_t>{21, 22, 10, 9, 5, 4}, // red
                               std::vector<uint8_t>{35}, // deep gray
                               std::vector<uint8_t>{20}); // light gray
            break;
        }
        case 17:
        {
            set_compressed_led(std::vector<uint8_t>{21, 22, 8, 9, 10, 5}, // red
                               std::vector<uint8_t>{20, 36}, // deep gray
                               std::vector<uint8_t>{35}); // light gray
            break;
        }
        case 18:
        {
            set_compressed_led(std::vector<uint8_t>{21, 22, 23, 10, 9, 8, 7,}, // red
                               std::vector<uint8_t>{20, 36}, // deep gray
                               std::vector<uint8_t>{35}); // light gray
            break;
        }
        case 19:
        {
            set_compressed_led(std::vector<uint8_t>{21, 22, 23, 9, 8, 7}, // red
                               std::vector<uint8_t>{19, 11}, // deep gray
                               std::vector<uint8_t>{20, 34}); // light gray
            break;
        }
        case 20:
        {
            set_compressed_led(std::vector<uint8_t>{21, 22, 23, 24, 25, 26}, // red
                               std::vector<uint8_t>{11, 19}, // deep gray
                               std::vector<uint8_t>{20, 34}); // light gray
            break;
        }
        case 21:
        {
            set_compressed_led(std::vector<uint8_t>{21, 22, 23, 32, 31, 30}, // red
                               std::vector<uint8_t>{11, 19}, // deep gray
                               std::vector<uint8_t>{20, 34}); // light gray
            break;
        }
        case 22:
        {
            set_compressed_led(std::vector<uint8_t>{21, 22, 33, 32, 31, 30}, // red
                               std::vector<uint8_t>{12}, // deep gray
                               std::vector<uint8_t>{20, 13}); // light gray
            break;
        }
        case 23:
        {
            set_compressed_led(std::vector<uint8_t>{21, 22, 33, 32, 31, 47}, // red
                               std::vector<uint8_t>{12}, // deep gray
                               std::vector<uint8_t>{13, 20}); // light gray
            break;
        }
        case 24:
        {
            set_compressed_led(std::vector<uint8_t>{21, 22, 33, 32, 46, 47}, // red
                               std::vector<uint8_t>{12}, // deep gray
                               std::vector<uint8_t>{20}); // light gray
            break;
        }
        case 25:
        {
            set_compressed_led(std::vector<uint8_t>{21, 34, 33, 45, 46}, // red
                               std::vector<uint8_t>{11, 22}, // deep gray
                               std::vector<uint8_t>{12, 20}); // light gray
            break;
        }
        case 26:
        {
            set_compressed_led(std::vector<uint8_t>{21, 34, 33, 45}, // red
                               std::vector<uint8_t>{11, 22}, // deep gray
                               std::vector<uint8_t>{20}); // light gray
            break;
        }
        default:
        {
            break;
        }
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
#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "hal/uart_types.h"
#include <esp_log.h>
#include <mutex>
#include <string>
#include <memory>
#include "gps_sentences.hpp"

namespace gps_sensor
{
class GTU8
{
public:
    GTU8();

    GTU8(const GTU8&) = default;

    ~GTU8();

    void init();

    void parse_complete_sentences(std::string& buffer);

private:
    void updateGPSData();

private:
    std::shared_ptr<GGA> gga_data_{nullptr};
    std::shared_ptr<GSV> gp_gsv_data_{nullptr};
    std::shared_ptr<GSV> bd_gsv_data_{nullptr};
    SemaphoreHandle_t data_mutex_;
};

void gpsTask(void *Params);
}
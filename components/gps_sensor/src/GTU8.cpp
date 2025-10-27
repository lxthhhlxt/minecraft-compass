#include "GTU8.hpp"
#include "driver/uart.h"
#include <cmath>
#include <vector>
#include "soc/gpio_num.h"
#include "status_manager.hpp"
#include "data_manager.hpp"
#include "esp_timer.h"

const static char* TAG = "GPS";

constexpr int read_buffer_size = 128; // 读取数据的缓冲区大小
constexpr int gps_freq = 10;

// 定义UART配置
constexpr uart_port_t uart_port_num       = UART_NUM_1;  // 使用的UART端口号
constexpr uint32_t    uart_baud_rate      = 9600;        // 串口设备的波特率
constexpr gpio_num_t  uart_rx_pin         = GPIO_NUM_18; // ESP32的RX引脚（接设备的TX）
constexpr gpio_num_t  uart_tx_pin         = GPIO_NUM_17; // ESP32的TX引脚（接设备的RX）
constexpr uint32_t    uart_rx_buffer_size = 1024;        // RX缓冲区大小
constexpr uint32_t    uart_tx_buffer_size = 0;           // TX缓冲区大小（若不发送可设为0）
constexpr uint32_t    uart_queue_size     = 10;          // 事件队列大小

void setup_uart();

namespace gps_sensor
{
GTU8::GTU8()
{
    data_mutex_ = xSemaphoreCreateMutex();
}

GTU8::~GTU8()
{
    vSemaphoreDelete(data_mutex_);
}

void GTU8::init()
{
    setup_uart();
}

void GTU8::parse_complete_sentences(std::string& buffer)
{
    size_t pos = 0;

    printf("start parse\n");
    while ((pos = buffer.find('\n', pos)) != std::string::npos)
    {
        std::string sentence = buffer.substr(0, pos + 1);
        buffer.erase(0, pos + 1);

        while (!sentence.empty() && (sentence.back() == '\n' || sentence.back() == '\r'))
        {
            sentence.pop_back();
        }

        if (!sentence.empty() && sentence[0] == '$')
        {
            // ESP_LOGI(TAG, "Sentence: %s\n\n", sentence.substr(0, 6).c_str());

            if (sentence.substr(0, 6) == "$GNGGA")
            {
                ESP_LOGI(TAG, "Sentence: %s\n\n", sentence.c_str());

                auto gga_ptr = std::make_shared<GGA>(sentence);
                ESP_LOGI(TAG, "GGA latitude: %f%s longitude: %f%s gps_status: %d satellite_count: %d\n",
                                gga_ptr->latitude_, gga_ptr->lat_direction_.c_str(),
                                gga_ptr->longitude_, gga_ptr->lon_direction_.c_str(),
                                static_cast<int>(gga_ptr->gps_status_), gga_ptr->satellite_count_);

                if (xSemaphoreTake(data_mutex_, pdMS_TO_TICKS(100)) == pdTRUE)
                {
                    gga_data_ = gga_ptr;
                    xSemaphoreGive(data_mutex_);
                }
            }
            else if (sentence.substr(0, 6) == "$GPGSV")
            {
                // ESP_LOGI(TAG, "Sentence: %s\n\n", sentence.c_str());

                auto gsv_ptr = std::make_shared<GSV>(sentence);
                ESP_LOGI(TAG, "GSV total_gsv_num: %d current_gsv_num: %d satellite_num: %d\n",
                              static_cast<int>(gsv_ptr->total_gsv_num_),
                              static_cast<int>(gsv_ptr->current_gsv_num_),
                              static_cast<int>(gsv_ptr->satellite_num_));

                if (xSemaphoreTake(data_mutex_, pdMS_TO_TICKS(100)) == pdTRUE)
                {
                    gp_gsv_data_ = gsv_ptr;
                    xSemaphoreGive(data_mutex_);
                }
            }
            else if (sentence.substr(0, 6) == "$BDGSV")
            {
                ESP_LOGI(TAG, "Sentence: %s\n\n", sentence.c_str());

                auto gsv_ptr = std::make_shared<GSV>(sentence);
                ESP_LOGI(TAG, "GSV total_gsv_num: %d current_gsv_num: %d satellite_num: %d\n",
                              static_cast<int>(gsv_ptr->total_gsv_num_),
                              static_cast<int>(gsv_ptr->current_gsv_num_),
                              static_cast<int>(gsv_ptr->satellite_num_));

                if (xSemaphoreTake(data_mutex_, pdMS_TO_TICKS(100)) == pdTRUE)
                {
                    bd_gsv_data_ = gsv_ptr;
                    xSemaphoreGive(data_mutex_);
                }
            }
        }

        pos = 0;
    }
    printf("end parse\n");
    updateGPSData();
}

void GTU8::updateGPSData()
{
    if (xSemaphoreTake(data_mutex_, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        if (!gga_data_ || !gp_gsv_data_ || !bd_gsv_data_)
        {
            xSemaphoreGive(data_mutex_);
            ESP_LOGI(TAG, "GGA or GSV data is not available.");
            return;
        }

        auto gps_status = StatusManager::getInstance().getGPSStatus();
        ESP_LOGI(TAG, "gps_status: %d, gga->gps_status: %d.", static_cast<int>(gps_status.value()), static_cast<int>(gga_data_->gps_status_));
        if ((gga_data_->gps_status_ == GPSStatus::NOT_POSITIONED ||
             gga_data_->gps_status_ == GPSStatus::UNKNOWN) &&
             gps_status.has_value() &&
             gps_status.value() != SensorStatus::CALIBRATING)
        {
            StatusManager::getInstance().setGPSStatus(SensorStatus::CALIBRATING);
            ESP_LOGI(TAG, "Set GPS status to Calibrating.");
        }
        else if ((gga_data_->gps_status_ != GPSStatus::NOT_POSITIONED &&
                  gga_data_->gps_status_ != GPSStatus::UNKNOWN) &&
                  gps_status.has_value() &&
                  gps_status.value() != SensorStatus::READY)
        {
            StatusManager::getInstance().setGPSStatus(SensorStatus::READY);
            ESP_LOGI(TAG, "Set GPS status to Ready.");
        }

        auto gps_data = GPSData();
        gps_data.is_valid = (gga_data_->gps_status_ != GPSStatus::NOT_POSITIONED &&
                             gga_data_->gps_status_ != GPSStatus::UNKNOWN);
        gps_data.coordinate.latitude = gga_data_->latitude_;
        gps_data.coordinate.longitude = gga_data_->longitude_;
        gps_data.satellite_num = std::max(gp_gsv_data_->satellite_num_, bd_gsv_data_->satellite_num_);

        DataManager::getInstance().setGPSData(gps_data);
        xSemaphoreGive(data_mutex_);
    }
}

void gpsTask(void *Params)
{
    auto gtu8 = GTU8();

    gtu8.init();

    uint8_t data[read_buffer_size]; // 数据缓冲区
    std::string buffer_;

    while (1)
    {
        int32_t start_time = esp_timer_get_time() / 1000;

        // 检查数据长度:cite[2]:cite[6]
        int length = 0;
        ESP_ERROR_CHECK(uart_get_buffered_data_len(uart_port_num, (size_t*)&length));
        if (length > 0)
        {
            // 读取数据，等待时间为100 ticks（可根据需要调整）:cite[2]:cite[6]
            int len = uart_read_bytes(uart_port_num, data, std::min(length, read_buffer_size), 100 / portTICK_PERIOD_MS);
            if (len > 0)
            {
                buffer_.append((char*)data, len);

                gtu8.parse_complete_sentences(buffer_);

                // 将接收到的数据打印为字符串（假设是文本数据）
                printf("Received: '%.*s'\n", len, (char*)data);
                // printf("Received: '%s'\n", (char*)data);
            }
        }

        int32_t end_time = esp_timer_get_time() / 1000;
        int32_t sleep_time = (1000 / gps_freq) - std::max(int32_t(0), end_time - start_time);
        vTaskDelay(pdMS_TO_TICKS(sleep_time)); // 稍作延迟，避免任务过于繁忙
    }
}
}

void setup_uart()
{
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
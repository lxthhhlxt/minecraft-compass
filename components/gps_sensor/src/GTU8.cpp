#include "GTU8.hpp"
#include "driver/uart.h"
#include <cmath>
#include <sstream>
#include <vector>
#include "status_manager.hpp"
#include "data_manager.hpp"

const char* TAG = "GPS";

constexpr int read_buffer_size = 128; // 读取数据的缓冲区大小

float dmToDegrees(const std::string& dm_str);

namespace gps_sensor
{
SentenceBase::SentenceBase(const std::string& sentence)
    : str_(sentence)
{}

bool SentenceBase::empty()
{
    return type_ == SentenceType::UNKNOWN;
}

GGA::GGA(const std::string& sentence)
    : SentenceBase(sentence)
{
    parse(str_);
}

bool GGA::parse(const std::string& sentence)
{
    if (sentence.substr(0, 6) != "$GNGGA")
    {
        ESP_LOGE(TAG, "Sentence type not GGA.");
        return false;
    }

    // 分割字符串
    std::vector<std::string> fields;
    std::stringstream ss(sentence);
    std::string field;

    while (std::getline(ss, field, ','))
    {
        fields.push_back(field);
    }

    // 检查字段数量（至少应有15个字段）
    if (fields.size() != 15)
    {
        ESP_LOGE(TAG, "Sentence parts less than 15.");
        return false;
    }

    // 时间（格式：hhmmss.sss）
    if (!fields[1].empty()) {
        utc_time_ = fields[1].substr(0, 2) + ":" +
                    fields[1].substr(2, 2) + ":" +
                    fields[1].substr(4, 2);
    }

    // 纬度（格式：ddmm.mmmm）
    latitude_ = dmToDegrees(fields[2]);
    lat_direction_ = fields[3];

    // 经度（格式：dddmm.mmmm）
    longitude_ = dmToDegrees(fields[4]);
    lon_direction_ = fields[5];

    // GPS状态
    if (!fields[6].empty())
    {
        gps_status_ = static_cast<GPSStatus>(std::stoi(fields[6]));
    }

    // 使用卫星数量
    if (!fields[7].empty()) {
        satellite_count_ = std::stoi(fields[7]);
    } else {
        satellite_count_ = 0;
    }

    // 水平精度因子
    if (!fields[8].empty()) {
        hdop_ = std::stod(fields[8]);
    } else {
        hdop_ = 99.9;
    }

    // 海平面高度
    if (!fields[9].empty()) {
        altitude_ = std::stod(fields[9]);
    } else {
        altitude_ = 0.0;
    }

    altitude_unit_ = fields[10];

    // 大地水准面高度
    if (!fields[11].empty()) {
        geoid_height_ = std::stod(fields[11]);
    } else {
        geoid_height_ = 0.0;
    }

    geoid_unit_ = fields[12];
    diff_time_ = fields[13];
    diff_station_id_ = fields[14].substr(0, fields[14].find('*')); // 去掉校验和

    type_ = SentenceType::GGA;

    return true;
}

GSV::GSV(const std::string& sentence)
    : SentenceBase(sentence)
{
    parse(sentence);
}

bool GSV::parse(const std::string& sentence)
{
    if (sentence.substr(0, 6) != "$GPGSV" && sentence.substr(0, 6) != "$BDGSV")
    {
        ESP_LOGE(TAG, "Sentence type not GSV.");
        return false;
    }

    // 分割字符串
    std::vector<std::string> fields;
    std::stringstream ss(sentence);
    std::string field;

    while (std::getline(ss, field, ','))
    {
        fields.push_back(field);
    }

    // 检查字段数量（至少应有3个字段）
    if (fields.size() != 4)
    {
        ESP_LOGE(TAG, "Sentence parts less than 4.");
        return false;
    }

    // GSV语句总数
    if (!fields[1].empty())
    {
        total_gsv_num_ = static_cast<uint8_t>(std::stoi(fields[1]));
    }

    // 当前GSV语句序号
    if (!fields[2].empty())
    {
        current_gsv_num_ = static_cast<uint8_t>(std::stoi(fields[2]));
    }

    // 当前卫星总数
    if (!fields[3].empty())
    {
        satellite_num_ = static_cast<uint8_t>(std::stoi(fields[3]));
    }
    return true;
}

GTU8::GTU8(const uart_port_t& uart_port)
    : uart_port_(uart_port)
{
    data_mutex_ = xSemaphoreCreateMutex();
}

GTU8::~GTU8()
{
    vSemaphoreDelete(data_mutex_);
}

void GTU8::init()
{}

std::shared_ptr<GGA> GTU8::getGGAData()
{
    if (xSemaphoreTake(data_mutex_, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        auto result = gga_data_;
        xSemaphoreGive(data_mutex_);
        return result;
    }
    return nullptr;
}

std::shared_ptr<GSV> GTU8::getGSVData()
{
    if (xSemaphoreTake(data_mutex_, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        auto result = gsv_data_;
        xSemaphoreGive(data_mutex_);
        return result;
    }
    return nullptr;
}

void GTU8::update()
{
    uint8_t data[read_buffer_size]; // 数据缓冲区
    std::string buffer_;

    while (1)
    {
        // 检查数据长度:cite[2]:cite[6]
        int length = 0;
        ESP_ERROR_CHECK(uart_get_buffered_data_len(uart_port_, (size_t*)&length));
        if (length > 0)
        {
            // 读取数据，等待时间为100 ticks（可根据需要调整）:cite[2]:cite[6]
            int len = uart_read_bytes(uart_port_, data, std::min(length, read_buffer_size), 100 / portTICK_PERIOD_MS);
            if (len > 0)
            {
                buffer_.append((char*)data, len);

                parse_complete_sentences(buffer_);

                updateGPSData();

                // 将接收到的数据打印为字符串（假设是文本数据）
                // printf("Received: '%.*s'\n", len, (char*)data);
                // printf("Received: '%s'\n", (char*)data);
            }
        }
        vTaskDelay(10 / portTICK_PERIOD_MS); // 稍作延迟，避免任务过于繁忙
    }
}

void GTU8::parse_complete_sentences(std::string& buffer)
{
    size_t pos = 0;

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
                ESP_LOGE(TAG, "获取data锁失败");
            }
            else if (sentence.substr(0, 6) == "$GPGSV" || sentence.substr(0, 6) == "$BDGSV")
            {
                ESP_LOGI(TAG, "Sentence: %s\n\n", sentence.c_str());

                auto gsv_ptr = std::make_shared<GSV>(sentence);
                ESP_LOGI(TAG, "GSV total_gsv_num: %d current_gsv_num: %d satellite_num: %d\n",
                              static_cast<int>(gsv_ptr->total_gsv_num_),
                              static_cast<int>(gsv_ptr->current_gsv_num_),
                              static_cast<int>(gsv_ptr->satellite_num_));

                if (xSemaphoreTake(data_mutex_, pdMS_TO_TICKS(100)) == pdTRUE)
                {
                    gsv_data_ = gsv_ptr;
                    xSemaphoreGive(data_mutex_);
                }
                ESP_LOGE(TAG, "获取data锁失败");
            }
        }

        pos = 0;
    }
}

void GTU8::updateGPSData()
{
    if (xSemaphoreTake(data_mutex_, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        auto gps_data = GPSData();
        xSemaphoreGive(data_mutex_);
    }
}

void gpsTask(void *Params)
{

}
}

// 将度分格式(ddmm.mmmm)转换为十进制度格式
float dmToDegrees(const std::string& dm_str)
{
    if (dm_str.empty())
    {
        return 0.0f;
    }

    // 找到小数点位置，分隔度和分
    size_t dot_pos = dm_str.find('.');
    if (dot_pos == std::string::npos || dot_pos < 2)
    {
        return 0.0f;
    }

    std::string deg_str = dm_str.substr(0, dot_pos - 2);
    std::string min_str = dm_str.substr(dot_pos - 2);
    ESP_LOGI(TAG, "Original: %s Degrees: %s Minutes: %s\n", dm_str.c_str(), deg_str.c_str(), min_str.c_str());

    float degrees = std::stof(deg_str);
    float minutes = std::stof(min_str);

    return degrees + minutes / 60.0f;
}
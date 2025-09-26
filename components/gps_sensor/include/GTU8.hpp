#pragma once

#include "hal/uart_types.h"
#include <esp_log.h>
#include <mutex>
#include <string>

namespace gps_sensor
{
struct Data
{
    float latitude{0.0f};
    float longitude{0.0f};
    int   gps_nums{0};
};

enum class SentenceType
{
    GGA,
    GLL,
    GSA,
    GSV,
    RMC,
    VTG,
    TXT,
    UNKNOWN
};

class SentenceBase
{
public:
    SentenceBase(const std::string& str);

    virtual ~SentenceBase() = default;

    bool empty();

protected:
    virtual bool parse(const std::string& sentence) = 0;

protected:
    SentenceType type_{SentenceType::UNKNOWN};
    std::string str_{""};
};

class GGA : public SentenceBase
{
public:
    GGA(const std::string& str);

    virtual ~GGA() override = default;

private:
    bool parse(const std::string& sentence) override;

public:
    std::string utc_time_;         // UTC时间
    std::string latitude_;         // 纬度
    std::string lat_direction_;    // 纬度方向
    std::string longitude_;        // 经度
    std::string lon_direction_;    // 经度方向
    int gps_status_;               // GPS状态
    int satellite_count_;          // 使用卫星数量
    double hdop_;                  // 水平精度因子
    double altitude_;              // 海平面高度
    std::string altitude_unit_;    // 高度单位
    double geoid_height_;          // 大地水准面高度
    std::string geoid_unit_;       // 大地水准面高度单位
    std::string diff_time_;        // 差分时间
    std::string diff_station_id_;  // 差分基站ID
};

class GTU8
{
public:
    GTU8(const uart_port_t& uart_port);

    GTU8(const GTU8&) = default;

    ~GTU8() = default;

    void init();

    Data getData();

    void update();

private:
    void parse_complete_sentences(std::string& buffer);

private:
    uart_port_t uart_port_;

    Data data_;
    std::mutex data_mutex_;
};
}
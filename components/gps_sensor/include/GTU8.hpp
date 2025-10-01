#pragma once

#include "hal/uart_types.h"
#include <esp_log.h>
#include <mutex>
#include <string>
#include <memory>

namespace gps_sensor
{
enum class GPSStatus
{
    NOT_POSITIONED = 0,      // 未定位
    SPS_MODE_VALID = 1,      // 无差分，SPS模式，定位有效
    DIFF_SPS_MODE_VALID = 2, // 带差分，SPS模式，定位有效
    PPS_MODE_VALID = 3,      // PPS模式，定位有效
    UNKNOWN = -1             // 未知状态
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

    GGA() = default;

    virtual ~GGA() override = default;

private:
    bool parse(const std::string& sentence) override;

public:
    std::string utc_time_;         // UTC时间
    float latitude_;               // 纬度
    std::string lat_direction_;    // 纬度方向
    float longitude_;              // 经度
    std::string lon_direction_;    // 经度方向
    GPSStatus   gps_status_{GPSStatus::NOT_POSITIONED}; // GPS状态
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

    std::shared_ptr<GGA> getData();

    void update();

private:
    void parse_complete_sentences(std::string& buffer);

private:
    uart_port_t uart_port_;

    std::shared_ptr<GGA> data_{nullptr};
    std::mutex data_mutex_;
};

void gpsTask(void *Params);
}
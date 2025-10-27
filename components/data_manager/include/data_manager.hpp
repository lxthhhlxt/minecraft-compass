#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <memory>

struct IMUData
{
    float yaw{0.0f};
    float pitch{0.0f};
    float roll{0.0f};
};

struct Coordinate
{
    float  latitude{0.0f};
    float  longitude{0.0f};
};

struct GPSData
{
    bool is_valid{false};
    Coordinate coordinate;
    int8_t satellite_num{0};
};

class DataManager
{
public:
    DataManager(const DataManager&) = delete;

    DataManager& operator=(const DataManager&) = delete;

    static DataManager& getInstance();

    void setIMUData(const IMUData& data);

    std::shared_ptr<IMUData> getLatestIMUData();

    void setGPSData(const GPSData& data);

    std::shared_ptr<GPSData> getLatestGPSData();

    void setDestLocation(const Coordinate& coordinate);

    std::shared_ptr<Coordinate> getLatestDestLocation();

private:
    DataManager();

    ~DataManager();

private:
    IMUData    latest_imu_data_;
    GPSData    latest_gps_data_;
    Coordinate latest_dest_location_;

    SemaphoreHandle_t imu_mutex_;
    SemaphoreHandle_t gps_mutex_;
    SemaphoreHandle_t dest_location_mutex_;
};
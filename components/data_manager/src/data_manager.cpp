#include "data_manager.hpp"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"

DataManager& DataManager::getInstance()
{
    static DataManager data_manager;
    return data_manager;
}

void DataManager::setIMUData(const IMUData& data)
{
    if (xSemaphoreTake(imu_mutex_, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        latest_imu_data_ = data;
        xSemaphoreGive(imu_mutex_);
    }
}

std::shared_ptr<IMUData> DataManager::getLatestIMUData()
{
    if (xSemaphoreTake(imu_mutex_, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        auto imu_data_ptr = std::make_shared<IMUData>(latest_imu_data_);
        xSemaphoreGive(imu_mutex_);
        return imu_data_ptr;
    }
    return nullptr;
}

void DataManager::setGPSData(const GPSData& data)
{
    if (xSemaphoreTake(gps_mutex_, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        latest_gps_data_ = data;
        xSemaphoreGive(gps_mutex_);
    }
}

std::shared_ptr<GPSData> DataManager::getLatestGPSData()
{
    if (xSemaphoreTake(gps_mutex_, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        auto gps_data_ptr = std::make_shared<GPSData>(latest_gps_data_);
        xSemaphoreGive(gps_mutex_);
        return gps_data_ptr;
    }
    return nullptr;
}

void DataManager::setDestLocation(const Coordinate& coordinate)
{
    if (xSemaphoreTake(dest_location_mutex_, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        latest_dest_location_ = coordinate;
        xSemaphoreGive(dest_location_mutex_);
    }
}

std::shared_ptr<Coordinate> DataManager::getLatestDestLocation()
{
    if (xSemaphoreTake(dest_location_mutex_, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        auto dest_location_ptr = std::make_shared<Coordinate>(latest_dest_location_);
        xSemaphoreGive(dest_location_mutex_);
        return dest_location_ptr;
    }
    return nullptr;
}

DataManager::DataManager()
{
    imu_mutex_ = xSemaphoreCreateMutex();
    gps_mutex_ = xSemaphoreCreateMutex();
    dest_location_mutex_ = xSemaphoreCreateMutex();
}

DataManager::~DataManager()
{
    vSemaphoreDelete(imu_mutex_);
    vSemaphoreDelete(gps_mutex_);
    vSemaphoreDelete(dest_location_mutex_);
}
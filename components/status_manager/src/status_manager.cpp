#include "status_manager.hpp"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"

StatusManager& StatusManager::getInstance()
{
    static StatusManager status_manager;
    return status_manager;
}

void StatusManager::setMPU6050Status(const SensorStatus &status)
{
    if (xSemaphoreTake(status_mutex_, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        mpu6050_status_ = status;
        xSemaphoreGive(status_mutex_);
    }
}

std::optional<SensorStatus> StatusManager::getMPU6050Status()
{
    if (xSemaphoreTake(status_mutex_, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        SensorStatus result = mpu6050_status_;
        xSemaphoreGive(status_mutex_);
        return result;
    }
    return std::nullopt;
}

void StatusManager::setQMC5883PStatus(const SensorStatus &status)
{
    if (xSemaphoreTake(status_mutex_, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        qmc5883p_status_ = status;
        xSemaphoreGive(status_mutex_);
    }
}

std::optional<SensorStatus> StatusManager::getQMC5883PStatus()
{
    if (xSemaphoreTake(status_mutex_, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        SensorStatus result = qmc5883p_status_;
        xSemaphoreGive(status_mutex_);
        return result;
    }
    return std::nullopt;
}

void StatusManager::setGPSStatus(const SensorStatus &status)
{
    if (xSemaphoreTake(status_mutex_, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        gps_status_ = status;
        xSemaphoreGive(status_mutex_);
    }
}

std::optional<SensorStatus> StatusManager::getGPSStatus()
{
    if (xSemaphoreTake(status_mutex_, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        auto result = gps_status_;
        xSemaphoreGive(status_mutex_);
        return result;
    }
    return std::nullopt;
}

void StatusManager::setDestLocationReceived(bool received)
{
    if (xSemaphoreTake(status_mutex_, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        dest_location_received_ = received;
        xSemaphoreGive(status_mutex_);
    }
}

bool StatusManager::isDestLocationReceived()
{
    if (xSemaphoreTake(status_mutex_, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        auto result = dest_location_received_;
        xSemaphoreGive(status_mutex_);
        return result;
    }
    return false;
}

DisplayState StatusManager::getDisplayState()
{
    DisplayState result{DisplayState::DISPLAY_NONE};

    if (xSemaphoreTake(status_mutex_, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        if (mpu6050_status_ == SensorStatus::CALIBRATING)
        {
            result = DisplayState::DISPLAY_MPU6050_CALIBRATING;
        }
        else if (qmc5883p_status_ == SensorStatus::CALIBRATING)
        {
            result = DisplayState::DISPLAY_QMC5883P_CALIBRATING;
        }
        else if (gps_status_ == SensorStatus::CALIBRATING)
        {
            result = DisplayState::DISPLAY_GPS_CALIBRATING;
        }
        else if (mpu6050_status_ == SensorStatus::READY &&
                 qmc5883p_status_ == SensorStatus::READY)
        {
            result = DisplayState::DISPLAY_COMPASS;

            if (gps_status_ == SensorStatus::READY &&
                dest_location_received_)
            {
                result = DisplayState::DISPLAY_DESTNATION;
            }
        }
        xSemaphoreGive(status_mutex_);
    }
    return result;
}

StatusManager::StatusManager()
{
    status_mutex_ = xSemaphoreCreateMutex();
}

StatusManager::~StatusManager()
{
    vSemaphoreDelete(status_mutex_);
}
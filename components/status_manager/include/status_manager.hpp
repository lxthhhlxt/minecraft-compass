#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <memory>

enum class DisplayState
{
    DISPLAY_NONE,
    DISPLAY_MPU6050_CALIBRATING,
    DISPLAY_QMC5883P_CALIBRATING,
    DISPLAY_GPS_CALIBRATING,
    DISPLAY_COMPASS,
    DISPLAY_DESTNATION
};

enum class SensorStatus
{
    NOT_INIT,
    CALIBRATING,
    READY,
};

class StatusManager
{
public:
    StatusManager(const StatusManager&) = delete;

    StatusManager& operator=(const StatusManager&) = delete;

    static StatusManager& getInstance();

    void setMPU6050Status(const SensorStatus &status);

    std::optional<SensorStatus> getMPU6050Status();

    void setQMC5883PStatus(const SensorStatus &status);

    std::optional<SensorStatus> getQMC5883PStatus();

    void setGPSStatus(const SensorStatus &status);

    std::optional<SensorStatus> getGPSStatus();

    void setDestLocationReceived(bool received);

    bool isDestLocationReceived();

    DisplayState getDisplayState();

private:
    StatusManager();

    ~StatusManager();

private:
    SensorStatus mpu6050_status_{SensorStatus::NOT_INIT};
    SensorStatus qmc5883p_status_{SensorStatus::NOT_INIT};
    SensorStatus gps_status_{SensorStatus::NOT_INIT};
    bool dest_location_received_{false};

    SemaphoreHandle_t status_mutex_;
};
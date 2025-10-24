#include "imu9dof.hpp"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "data_manager.hpp"
#include <cmath>

extern "C"
{
#include "MahonyAHRS.h"
}

static const char *TAG = "IMU9DoF";

// 定义I2C配置
constexpr gpio_num_t mpu6050_scl_io   = GPIO_NUM_4;
constexpr gpio_num_t mpu6050_sda_io   = GPIO_NUM_5;
constexpr uint32_t   mpu6050_freq_hz  = 400000;
constexpr i2c_port_t mpu6050_i2c_num  = I2C_NUM_0;
constexpr uint8_t    mpu6050_addr     = 0x68;
constexpr i2c_port_t qmc5883p_i2c_num = I2C_NUM_0;
constexpr uint8_t    qmc5883p_addr    = 0x2C;
constexpr uint16_t   imu_freq         = 100;

void compensate_tilt_and_declination(imu_sensor::Data& data, float declination_rad);
imu_sensor::EulerAngles quaternionToEuler(const imu_sensor::Quaternion& q);
double degToRad(double deg);
double radToDeg(double radians);

// 初始化I2C主机
static void i2c_master_init()
{
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = mpu6050_sda_io;
    conf.scl_io_num = mpu6050_scl_io;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = mpu6050_freq_hz;
    conf.clk_flags = 0;

    esp_err_t err = i2c_param_config(mpu6050_i2c_num, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C参数配置失败: %d", err);
        return;
    }

    err = i2c_driver_install(mpu6050_i2c_num, conf.mode, 0, 0, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C驱动安装失败: %d", err);
    }
}

// I2C扫描函数
void i2c_scanner()
{
    ESP_LOGI(TAG, "Scanning I2C bus...\n");
    for (uint8_t addr = 1; addr < 127; addr++)
    {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(mpu6050_i2c_num, cmd, pdMS_TO_TICKS(100));
        i2c_cmd_link_delete(cmd);

        if (ret == ESP_OK)
        {
            ESP_LOGI(TAG, "Device found at address 0x%02X\n", addr);
        }
    }
    ESP_LOGI(TAG, "Scan completed.\n");
}

namespace imu_sensor
{
IMU9DoF::IMU9DoF(i2c_port_t mpu6050_port, uint8_t mpu6050_address,
                 i2c_port_t qmc5883p_port, uint8_t qmc5883p_address)
{
    mpu6050_ = std::make_shared<MPU6050::MPU6050>(mpu6050_port, mpu6050_address);
    qmc5883p_ = std::make_shared<QMC5883P::QMC5883P>(qmc5883p_port, qmc5883p_address);
}

esp_err_t IMU9DoF::init()
{
    if (auto ret = mpu6050_->init(); ret != ESP_OK)
    {
        ESP_LOGE(TAG, "mpu6050 初始化失败");
        return ret;
    }

    if (auto ret = qmc5883p_->init(); ret != ESP_OK)
    {
        ESP_LOGE(TAG, "qmc5883p 初始化失败");
        return ret;
    }

    return ESP_OK;
}

void IMU9DoF::calc9AsixData()
{
    auto mpu6050_data_opt = mpu6050_->get_cal_data();
    if (mpu6050_data_opt == std::nullopt)
    {
        ESP_LOGE(TAG, "Get mpu6050 data failed.");
        return;
    }

    auto qmc5883p_data_opt = qmc5883p_->get_cal_data();
    if (qmc5883p_data_opt == std::nullopt)
    {
        ESP_LOGE(TAG, "Get qmc5883p data failed.");
        return;
    }

    Data device_asix_data;
    device_asix_data.accel_x = -mpu6050_data_opt.value()->accel_x;
    device_asix_data.accel_y = -mpu6050_data_opt.value()->accel_y;
    device_asix_data.accel_z = mpu6050_data_opt.value()->accel_z;

    device_asix_data.gyro_x = -mpu6050_data_opt.value()->gyro_x;
    device_asix_data.gyro_y = -mpu6050_data_opt.value()->gyro_y;
    device_asix_data.gyro_z = mpu6050_data_opt.value()->gyro_z;

    device_asix_data.mag_x = -qmc5883p_data_opt.value()->y;
    device_asix_data.mag_y = qmc5883p_data_opt.value()->x;
    device_asix_data.mag_z = qmc5883p_data_opt.value()->z;

    // ESP_LOGI(TAG, "加速度: X=%.2f, Y=%.2f, Z=%.2f",
    //          device_asix_data.accel_x, device_asix_data.accel_y, device_asix_data.accel_z);
    // ESP_LOGI(TAG, "陀螺仪: X=%.2f, Y=%.2f, Z=%.2f",
    //          device_asix_data.gyro_x, device_asix_data.gyro_y, device_asix_data.gyro_z);
    // ESP_LOGI(TAG, "磁力计: X=%.2f,\tY=%.2f,\tZ=%.2f",
    //          device_asix_data.mag_x, device_asix_data.mag_y, device_asix_data.mag_z);

    compensate_tilt_and_declination(device_asix_data, -0.057f);

    MahonyAHRSupdate(degToRad(device_asix_data.gyro_x), degToRad(device_asix_data.gyro_y), degToRad(device_asix_data.gyro_z),
                     device_asix_data.accel_x, device_asix_data.accel_y, device_asix_data.accel_z,
                     device_asix_data.mag_x, device_asix_data.mag_y, device_asix_data.mag_z);

    Quaternion q;
    q.w = q0;
    q.x = q1;
    q.y = q2;
    q.z = q3;

    // 转换为欧拉角
    EulerAngles euler = quaternionToEuler(q);
    IMUData imu_data;
    imu_data.yaw = radToDeg(euler.yaw);
    imu_data.pitch = radToDeg(euler.pitch);
    imu_data.roll = radToDeg(euler.roll);
    DataManager::getInstance().setIMUData(imu_data);

    // static int count = 0;
    // if (count++ % 10 == 0)
    // {
    //     ESP_LOGI(TAG, "欧拉角: Yaw=%.2f, Pitch=%.2f, Roll=%.2f",
    //              radToDeg(euler.yaw), radToDeg(euler.pitch), radToDeg(euler.roll));
    // }
}

void imuTask(void *Params)
{
    i2c_master_init();

    i2c_scanner();

    imu_sensor::IMU9DoF imu(mpu6050_i2c_num,  mpu6050_addr,
                            qmc5883p_i2c_num, qmc5883p_addr);
    imu.init();

    while (1)
    {
        int32_t start_time = esp_timer_get_time() / 1000;
        imu.calc9AsixData();
        int32_t end_time = esp_timer_get_time() / 1000;
        int32_t sleep_time = (1000 / imu_freq) - std::max(int32_t(0), (end_time - start_time));
        vTaskDelay(pdMS_TO_TICKS(sleep_time));
    }
}
}

void compensate_tilt_and_declination(imu_sensor::Data& data, float declination_rad)
{

    // 1. 归一化加速度计数据
    float norm = sqrtf(data.accel_x * data.accel_x +
                       data.accel_y * data.accel_y +
                       data.accel_z * data.accel_z);

    if (norm < 0.001f) {
        // 避免除以零，返回未处理的磁力计数据
        return;
    }

    float ax_norm = data.accel_x / norm;
    float ay_norm = data.accel_y / norm;
    float az_norm = data.accel_z / norm;

    // 2. 使用加速度计数据计算俯仰角(pitch)和横滚角(roll)
    // 横滚角 (绕X轴旋转)
    float roll = atan2f(ay_norm, az_norm);

    // 俯仰角 (绕Y轴旋转)
    float pitch = atan2f(-ax_norm, sqrtf(ay_norm * ay_norm + az_norm * az_norm));

    // 3. 计算旋转矩阵的元素（从机体坐标系到地平坐标系）
    float sin_roll = sinf(roll);
    float cos_roll = cosf(roll);
    float sin_pitch = sinf(pitch);
    float cos_pitch = cosf(pitch);

    // 4. 对磁力计数据进行倾斜补偿（从机体坐标系转换到地平坐标系）
    // 水平分量X (北方向)
    float hx = data.mag_x * cos_pitch +
               data.mag_y * sin_roll * sin_pitch +
               data.mag_z * cos_roll * sin_pitch;

    // 水平分量Y (东方向)
    float hy = data.mag_y * cos_roll -
               data.mag_z * sin_roll;

    // 垂直分量Z (地方向)
    float hz = -data.mag_x * sin_pitch +
               data.mag_y * sin_roll * cos_pitch +
               data.mag_z * cos_roll * cos_pitch;

    // 5. 对水平磁场分量进行磁偏角校正（从磁北到真北）
    float north = hx * cosf(declination_rad) - hy * sinf(declination_rad);
    float east = hx * sinf(declination_rad) + hy * cosf(declination_rad);
    float down = hz; // 垂直分量不变

    // 6. 返回结果
    data.mag_x = north;
    data.mag_y = east;
    data.mag_z = down;
}

// 将四元数转换为欧拉角（ZYX顺序）
imu_sensor::EulerAngles quaternionToEuler(const imu_sensor::Quaternion& q)
{
    imu_sensor::EulerAngles angles;

    // 计算滚转 (x轴旋转)
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles.roll = std::atan2(sinr_cosp, cosr_cosp);

    // 计算俯仰 (y轴旋转)
    double sinp = 2 * (q.w * q.y - q.z * q.x);
    if (std::abs(sinp) >= 1)
        angles.pitch = std::copysign(M_PI / 2, sinp); // 使用90度如果超出范围
    else
        angles.pitch = std::asin(sinp);

    // 计算偏航 (z轴旋转)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);

    return angles;
}


// 将角度转换为弧度
double degToRad(double deg)
{
    return deg * M_PI / 180.0;
}

// 将弧度转换为角度
double radToDeg(double radians)
{
    return radians * 180.0 / M_PI;
}